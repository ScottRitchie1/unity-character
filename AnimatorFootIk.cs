using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using TMPro;
using Unity.Mathematics;
using Unity.Services.Mediation;
using UnityEditor;
using UnityEngine;
using static AnimatorFootIk.footReference;

[RequireComponent(typeof(Animator))]
public class AnimatorFootIk : MonoBehaviour
{
    internal class footReference
    {
        internal enum FootIkTarget {right, left};
        FootIkTarget m_foot;
        //the foot bone transform
        Transform m_bone;
        //the height from the foot bone transform to the bottom of the foot
        float m_bottomHeight;
        //the right direction of the foot
        Vector3 m_localRightDirection;
        
        //the position the raycast calculates for ik
        internal Vector3 ikPositionTarget;
        //the rotation the raycast calculates for ik
        internal Quaternion ikRotationTarget;

        //the vertical offset of the ik foot position
        internal float ikTargetVerticalOffset;

        internal FootIkTarget foot => m_foot;
        internal Transform bone => m_bone;
        internal float bottomHeight => m_bottomHeight;
        internal Vector3 localRightDirection => m_localRightDirection;
        internal AvatarIKGoal avatarIKGoal => m_foot == FootIkTarget.right ? AvatarIKGoal.RightFoot : AvatarIKGoal.LeftFoot;

        internal footReference(FootIkTarget _foot, Animator _animator)
        {
            m_foot = _foot;
            m_bone = _animator.GetBoneTransform(m_foot == FootIkTarget.right ? HumanBodyBones.RightFoot : HumanBodyBones.LeftFoot);
            m_localRightDirection = m_bone.InverseTransformDirection(_animator.transform.right);
            m_bottomHeight = m_foot ==  FootIkTarget.right ? _animator.rightFeetBottomHeight : _animator.leftFeetBottomHeight;
        }

        //gets the height for a step. calculated using the lenght of the leg
        internal float GetLegStepHeight(Animator _animator)
        { 
            return Vector3.Distance(m_bone.position, _animator.GetBoneTransform(m_foot == FootIkTarget.right ? HumanBodyBones.RightUpperLeg : HumanBodyBones.LeftUpperLeg).position);
        }

    }

    footReference rightFoot;
    footReference leftFoot;
    //the offset of the 
    float rootOffset = 0;
    Animator animator;//animator ref

    [SerializeField]
    float stepHeight = 0.5f;//height above and below the root we raycast for feet

    public float GetStepHeight => stepHeight;
    public float GetRootOffset => rootOffset;

    //the amount the hips positions is damped when re-adjusting its height.
    [SerializeField, Min(0)]
    float damping = 4f;

    //the layers for objects which the feet raycast collide with
    [SerializeField]
    LayerMask footLayerMask = ~0;

    //the Ik weights
    [Range(0f,1f)]
    public float rightFootWeight = 1;
    [Range(0f, 1f)]
    public float leftFootWeight = 1;
    [Range(0f, 1f)]
    public float bodyWeight = 1;

    // Start is called before the first frame update
    void Awake()
    {
        animator = GetComponent<Animator>();

        if (!animator.isHuman)
        {
            enabled = false;
            return;
        }

        rightFoot = new footReference(FootIkTarget.right, animator);
        leftFoot = new footReference(FootIkTarget.left, animator);

        stepHeight = rightFoot.GetLegStepHeight(animator);
    }

    private void OnAnimatorIK(int layerIndex)
    {   
        //Calculates the feets target Position and Rotation for Ik
        CalculateFootIk(rightFoot);
        CalculateFootIk(leftFoot);

        //Sets the Root Offset based on the lowest foot target position
        Vector3 hipsOffset;
        SetRootOffset(rightFoot.ikTargetVerticalOffset, leftFoot.ikTargetVerticalOffset, out hipsOffset);

        //Sets the feet target positions and rotations
        SetFootIk(rightFoot, hipsOffset, rightFootWeight);
        SetFootIk(leftFoot, hipsOffset, leftFootWeight);
    }


    void CalculateFootIk(footReference footReference)
    {
        //Calculate the position the right foot should cast a ray from
        Vector3 footPosRelativeToRootXZ = transform.InverseTransformPoint(footReference.bone.position);
        footPosRelativeToRootXZ.y = 0;
        Vector3 footCastPos = transform.TransformPoint(footPosRelativeToRootXZ) + transform.up * stepHeight;

        //raycast our foot
        RaycastHit footRaycastHit;
        if (Physics.Raycast(footCastPos, -transform.up, out footRaycastHit, stepHeight * 2, footLayerMask, QueryTriggerInteraction.Ignore))
        {

            //calculate the position the foot position should match based on our hit position
            footReference.ikPositionTarget = footRaycastHit.point + footRaycastHit.normal * footReference.bottomHeight;

            //calculate the rotation the foot should match based on the hit normal
            Vector3 forward = Vector3.Cross(footRaycastHit.normal, -footReference.bone.TransformDirection(footReference.localRightDirection));
            //Vector3 forward = Vector3.Cross(footRaycastHit.normal, -footTransform.forward);
            footReference.ikRotationTarget = Quaternion.LookRotation(forward, footRaycastHit.normal);
            //footTargetRot = Quaternion.FromToRotation(transform.up, footRaycastHit.normal) * transform.rotation;


            footReference.ikTargetVerticalOffset = footRaycastHit.distance - stepHeight;
            return;
            
        }

        //if the foot raycast didnt hit anything, or the target position was below the original, then perform no ik
        footReference.ikPositionTarget = footReference.bone.position;
        footReference.ikRotationTarget = footReference.bone.rotation;
        footReference.ikTargetVerticalOffset = 0;
    }

    void SetFootIk(footReference footReference, Vector3 hipsOffset, float weight)
    {
        //check if the ik target foot position is above the original foot position
        bool isIKPosAboveOriginalPos = Vector3.Dot(transform.up, (footReference.bone.position + hipsOffset) - footReference.ikPositionTarget) < 0;

        //if the target position is above the original, then we need to ik
        if (isIKPosAboveOriginalPos)
        {
            //set ik weights and positions
            animator.SetIKPositionWeight(footReference.avatarIKGoal, weight);
            animator.SetIKRotationWeight(footReference.avatarIKGoal, weight);
            animator.SetIKPosition(footReference.avatarIKGoal, footReference.ikPositionTarget);
            animator.SetIKRotation(footReference.avatarIKGoal, footReference.ikRotationTarget);
        }
        else
        {
            //if target position is below the actual position then we dont perform ik
            animator.SetIKPositionWeight(footReference.avatarIKGoal, 0);
            animator.SetIKRotationWeight(footReference.avatarIKGoal, 0);
        }
    }

    private void SetRootOffset(float rightFootTargetOffset, float leftFootTargetOffset, out Vector3 hipsOffset)
    {
        //Calculate the amount of offset our root should have
        float lowestTargetOffset = Mathf.Max(rightFootTargetOffset, leftFootTargetOffset);
        lowestTargetOffset = Mathf.Lerp(0, lowestTargetOffset, bodyWeight);

        //damp towards our target offset
        rootOffset = Mathf.MoveTowards(rootOffset, lowestTargetOffset, Time.deltaTime * damping);

        //Calculate the offset direction and set the offset
        hipsOffset = -transform.up * rootOffset;
        animator.bodyPosition += hipsOffset;
    }
    
}
