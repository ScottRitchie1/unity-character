
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// FIXXXXXXXX
/// implement step height
/// max walkable angle
/// use jetpack after falling - currently have to press space twice to work
/// </summary>
/// 


[System.Serializable]
public class PlayerEngineControlVariables
{
    //MoveSpeed of the character
    public float maxSpeed = 7.5f;
    //turn speed of the character
    public float turnRate = 5;
    //acceleration of the ccharacter
    public float acceleration = 3;
    //deacceleration of the character
    public float deacceleration = 5;
    //rate at which the characcter corrects its move direction
    public float moveDirectionSmoothing = 0.2f;

    public PlayerEngineControlVariables(float _maxSpeed = 7.5f, float _turnRate = 5, float _acceleration = 3, float _deacceleration = 5, float _moveDirectionSmoothing = 0.2f)
    {
        maxSpeed = _maxSpeed;
        turnRate = _turnRate;
        acceleration = _acceleration;
        deacceleration = _deacceleration;
        moveDirectionSmoothing = _moveDirectionSmoothing;
    }
}

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(CapsuleCollider))]
public class PlayerEngine : MonoBehaviour {
    bool m_isGrounded = false;
    bool m_isSlipping = false;
    

    #region Component References
    Rigidbody m_rb;// the controllers rigid body
    Animator m_anim;// the controllers animator component
    CapsuleCollider m_collider;// the collider of the controller
    #endregion

    #region Control Variables
    //Control variables

    public PlayerEngineControlVariables controlVariables;

    // the target rotation for a character
    Vector3 m_targetForwards;
    // the target direction we want to travel
    Vector3 m_targetMoveDirection;

    // external force applied in the fixed update - ie jumping, knockback
    Vector3 nextFixedUpdateForce = Vector3.zero;
    // the calculated move force to move around our charater in fixed update
    Vector3 fixedUpdateMoveForce;
    #endregion

    #region Grounded Variables
    [Header("Step Settings")]
    public bool useStepHeights = true;
    //how high the character will step
    public float stepHeight = 0.5f;
    // the distance we detect steps from
    public float stepDistance = 1;
    float stepHeightRadiusOffset;
    
    [Space]
    //ground layers
    public LayerMask groundLayerMask;
    [Space]
    // the curve to determine the movement speed weight we use on a spesific incline.
    //also used to determine the max walk and max ground angles
    // must always have at least 3 keys
    public AnimationCurve inclineMoveSpeedLimiterCurve = new AnimationCurve(new Keyframe(0, 10), new Keyframe(15, 10, 0, -0.0115f), new Keyframe(45, 0, 0, 0), new Keyframe(60, 0));

    // the distance to the ground
    float m_distanceToGround = 0;

    //the radius we use to check if we are grounded
    float m_groundedCheckRadius;

    //max surface incline we can move on
    float m_maxMoveIncline;
    // max surface incline we can stay grounded on
    float m_maxGroundedIncline;

    // the max speed we can travel while on an incline
    float maxMoveSpeedOnCurrentIncline;

    //Vector Pointing paralell to the grounded incline with its forward towards the move direction
    Vector3 inclineMoveVector;
    //Vector pointing down the grounded incline
    Vector3 downInclineVector;
    // the signed angle of the inclineMoveVector
    float moveInclineAngle;
    //the abs angle of the downInclineVector;
    float downInclineAngle;

    // the raycast hit for the ground
    RaycastHit m_groundHit;// the place we hit the ground
    #endregion

    #region Collision
    [Header("Collision"), Range(0.0f, 1.0f)]
    public float colliderHeadPadding = 0.25f;
    #endregion

    #region Properties
    public new Rigidbody rb
    { get { return m_rb; } }
    public new CapsuleCollider collider
    { get { return m_collider; } }
    public Animator anim
    { get { return m_anim; } }
    public bool isGrounded
    { get { return m_isGrounded; } }
    public bool isSlipping
    { get { return m_isSlipping; } }
    public float distanceToGround
    { get { return m_distanceToGround; } }
    public Vector3 slippingVector
    { get { return downInclineVector; } }
    #endregion

    #region Events
    public delegate void OnForceDelegate(Vector3 force);
    public OnForceDelegate OnForceAppliedEvent;

    public delegate void OnGroundDelegate(bool isGrounded);
    public OnGroundDelegate OnGroundEvent;
    #endregion

    List<RaycastHit> stepHits = new List<RaycastHit>();

    protected virtual void Awake()
    {
        m_anim = GetComponentInChildren<Animator>();
        if(m_anim)
            m_anim.applyRootMotion = false;
        m_rb = GetComponent<Rigidbody>();// grab rigid body
        m_rb.interpolation = RigidbodyInterpolation.Interpolate;


        m_collider = GetComponent<CapsuleCollider>();
        m_groundedCheckRadius = m_collider.radius;
        m_maxGroundedIncline = inclineMoveSpeedLimiterCurve.keys[inclineMoveSpeedLimiterCurve.keys.Length - 1].time;
        m_maxMoveIncline = inclineMoveSpeedLimiterCurve.keys[inclineMoveSpeedLimiterCurve.keys.Length - 2].time;

        m_targetMoveDirection = transform.forward;
        m_targetForwards = Vector3.forward;
    }

    protected virtual void Start () {
        OnGrounded();// default to grounded state
        stepHeightRadiusOffset = CalculateStepHeightRadiusOffset();
    }

    protected virtual void Update()
    {
        //If we are grounded
        if (m_isGrounded)
            UpdateGrounded();
        else// if we not grounded OR we are preparing to jump
            UpdateNotGrounded();

        //UpdateInputs();// update inputs
        UpdateControllers();

        UpdateRotationMovement();// update rotation
        UpdatePositionMovement();// update movement
    }

    protected virtual void FixedUpdate()
    {

        if (m_isSlipping)
            FixedUpdateCharacterSlipping();//apply slipping force
        else
            FixedUpdateCharacterMovement();//apply movement force

        //forces - ie jump, knockback
        if (nextFixedUpdateForce.magnitude > 0)
            FixedUpdateApplyForce();

        
    }

    protected virtual void LateUpdate()
    {
        CalculateCollider();
    }

    public virtual void UpdateControllers()
    {
        Debug.LogError("Player Engine controllers not updating.", this);
    }

    private void UpdateGrounded()
    {
        float error = 0.2f;
        RaycastHit rayDirectlyDownHit;
        //sets grounede to false if: we are actually not grounded, we are jumping or our step angle is above the required
        if (!Physics.SphereCast(m_rb.position + (Vector3.up * (m_groundedCheckRadius + error)), m_groundedCheckRadius, Vector3.down, out m_groundHit, stepHeight + error, groundLayerMask, QueryTriggerInteraction.Ignore)
            || nextFixedUpdateForce.magnitude > 0)//|| GetCurrentStepAngle() > maxUngroundAngle)
        {
            OnUngrounded();
            return;
        }else if(Physics.Raycast(m_rb.position + Vector3.up * error, Vector3.down, out rayDirectlyDownHit, stepHeight + error, groundLayerMask, QueryTriggerInteraction.Ignore))
        {
            
            m_groundHit = rayDirectlyDownHit;
        }

        //calculate the distance from the ground
        m_distanceToGround = CalculateDistanceToGround();
        //calculate our surface decline
        downInclineAngle = CalculateDownInclineAngle();


        //check if we are slipping or not
        if (downInclineAngle < m_maxMoveIncline)
            m_isSlipping = false;
        else
            m_isSlipping = true;

        //Update our position to the calculated height - step height
        if (!m_isSlipping && useStepHeights)
        {
            m_rb.position = new Vector3(m_rb.position.x, GetStepHeight(error), m_rb.position.z);
        }

    }

    private void UpdateNotGrounded()
    {
        // if our check determines we are grounded this frame
        if (Physics.SphereCast(m_rb.position + Vector3.up * m_groundedCheckRadius, m_groundedCheckRadius * 0.9f, Vector3.down, out m_groundHit, Mathf.Infinity, groundLayerMask, QueryTriggerInteraction.Ignore))
        {
            m_distanceToGround = CalculateDistanceToGround();
            // if we would collide with the ground next frame
            if (m_distanceToGround-0.1f <= -m_rb.velocity.y * Time.deltaTime)
            {
                //recalculate the incline of the hit surface
                downInclineAngle = CalculateDownInclineAngle();
                if (downInclineAngle < m_maxGroundedIncline)
                {
                    OnGrounded();
                    return;
                }
            }
        }
        else
        {
            m_distanceToGround = Mathf.Infinity;
        }
    }

    private void UpdatePositionMovement()
    {
        float moveSpeed;
        Vector3 moveDirection;
        Vector3 velocity;

        if (m_isGrounded)
        {
            // scale the move velocity depending on the incline
            maxMoveSpeedOnCurrentIncline = CalculateMoveSpeedLimitFromIncline();
            //velocity should be the normal velocity of the rigidbody - since we can move up and down inclines
            velocity = m_rb.velocity;
            // move drection should be the forward direction of the incline
            moveDirection = inclineMoveVector.normalized;
        }
        else
        {
            //Incline shouldnt have an effect on movement while not grounded
            maxMoveSpeedOnCurrentIncline = controlVariables.maxSpeed;
            //use the xz velocity becasue we down want our y velocity to interfear with our speed movement while not grounded
            velocity = new Vector3(m_rb.velocity.x, 0, m_rb.velocity.z);
            // move direction should be our target direction on the x/z axis
            moveDirection = m_targetMoveDirection;
        }

        float speedChangeRate = ((m_rb.velocity.magnitude < controlVariables.maxSpeed ? controlVariables.acceleration : controlVariables.deacceleration));
        //lerps our current speed towards our target speed
        moveSpeed = Mathf.MoveTowards(velocity.magnitude, Mathf.Clamp(controlVariables.maxSpeed, 0, maxMoveSpeedOnCurrentIncline), speedChangeRate * Time.deltaTime);

        //slerps our current travel direction to our target travel direction
        moveDirection = Vector3.Slerp(velocity.normalized, moveDirection, controlVariables.moveDirectionSmoothing * Time.deltaTime);
        //calculate our force to move us in the direction and align us to our forwards
        fixedUpdateMoveForce = (moveDirection * moveSpeed);//
    }

    private void UpdateRotationMovement()
    {
        float angle = Vector3.SignedAngle(transform.forward, Vector3.Slerp(transform.forward, m_targetForwards, controlVariables.turnRate * Time.deltaTime), Vector3.up);
        transform.RotateAround(transform.position, Vector3.up, angle);
    }

    private void FixedUpdateCharacterMovement()
    {
        //move our character
        m_rb.AddForce(fixedUpdateMoveForce - new Vector3(m_rb.velocity.x, (m_isGrounded ? m_rb.velocity.y : 0), m_rb.velocity.z), ForceMode.VelocityChange);
    }

    private void FixedUpdateCharacterSlipping()
    {
        downInclineVector = CalculateDownInclineVector();
        if(m_rb.velocity.magnitude < controlVariables.maxSpeed)
            m_rb.AddForce(downInclineVector * controlVariables.acceleration, ForceMode.Acceleration);//move our character
    }

    private void FixedUpdateApplyForce()
    {
        //apply force
        m_rb.AddForce(nextFixedUpdateForce, ForceMode.Impulse);
        if(OnForceAppliedEvent != null)
        {
            OnForceAppliedEvent(nextFixedUpdateForce);//event
        }
        nextFixedUpdateForce = Vector3.zero;
        OnUngrounded();//unground
    }

    public void AddForce(Vector3 force)
    {
        nextFixedUpdateForce = force;
    }

    public void SetTargetForwards(Vector3 forwards)
    {
        m_targetForwards = forwards.normalized;
    }

    public void SetMoveDirection(Vector3 direction)
    {
        m_targetMoveDirection = direction.normalized;
    }

    private void OnGrounded()
    {
        m_isGrounded = true;
        m_rb.useGravity = false;
        m_distanceToGround = 0;
        if(OnGroundEvent != null)
        {
            OnGroundEvent(m_isGrounded);
        }
        m_rb.velocity = new Vector3(m_rb.velocity.x, 0, m_rb.velocity.z);

    }

    private void OnUngrounded()
    {
        // when we transition from grouneded to not grounede
        m_isGrounded = false;
        m_rb.useGravity = true;
        m_isSlipping = false;
        maxMoveSpeedOnCurrentIncline = 0;
        if (OnGroundEvent != null)
        {
            OnGroundEvent(m_isGrounded);
        }
    }

    private void CalculateCollider()
    {
        if (m_anim)
        {
            m_collider.height = Mathf.Clamp(m_anim.GetBoneTransform(HumanBodyBones.Head).transform.position.y - m_rb.position.y + colliderHeadPadding, m_collider.radius*2, float.MaxValue);
            m_collider.center = Vector3.up * m_collider.height * 0.5f;
        }
    }

    private float CalculateDistanceToGround()
    {
        return Mathf.Abs(m_rb.position.y - m_groundHit.point.y);
    }

    private float CalculateDownInclineAngle()
    {
        return Vector3.Angle(Vector3.up, m_groundHit.normal);
    }

    private float CalculateMoveInclineAngle()
    {
        // calculates the forward vector along the ground surface
        inclineMoveVector = CalculateInclineMoveVector();
        //angle along the incline
        return Vector3.SignedAngle((m_isGrounded ? inclineMoveVector : m_targetMoveDirection), m_targetMoveDirection, Vector3.Cross(Vector3.up, m_targetMoveDirection));
    }

    private Vector3 CalculateInclineMoveVector()
    {
        // the vector pointing forward along the ground surface
        return Vector3.Cross(Vector3.Cross(Vector3.up, m_targetMoveDirection), m_groundHit.normal);
    }

    private Vector3 CalculateDownInclineVector()
    {
        // this vector points straight down the incline surface
        return Vector3.Cross(Vector3.Cross(Vector3.up, m_groundHit.normal), m_groundHit.normal);
    }

    private float CalculateMoveSpeedLimitFromIncline()
    {
        moveInclineAngle = CalculateMoveInclineAngle();
        // if facing up, use surface incline if facing down, use 0
        return Mathf.Clamp(inclineMoveSpeedLimiterCurve.Evaluate(Mathf.Clamp(moveInclineAngle, 0, float.MaxValue)), 0, controlVariables.maxSpeed);
    }

    //calculates the offset radius to have the player collider just brush past the step edge when walking up
    float CalculateStepHeightRadiusOffset()
    {
        //credit to /r/chemcas
        //https://www.reddit.com/r/askmath/comments/ciyonl/looking_for_help_solving_this_math_problem_not/
        float height = stepHeight;
        float lenght = stepDistance;
        float radius = m_collider.radius;

        float b = (radius * ((Mathf.Pow(lenght, 2) + Mathf.Pow(height, 2) - (height*radius)) - lenght * Mathf.Sqrt(Mathf.Pow(lenght, 2) - (2*height*radius) + Mathf.Pow(height, 2)))) / (Mathf.Pow(height, 2) + Mathf.Pow(radius, 2) + Mathf.Pow(lenght, 2) - (2*height*radius) );
        float a = Mathf.Sqrt(Mathf.Pow(radius, 2) - Mathf.Pow(radius - b, 2));
        float m = a / (radius - b);
        float c = height - lenght * m;

        return -(c / m);
    }

    #region DEBUG
    private void OnDrawGizmos()
    {
        
        Gizmos.color = Color.black;
        Vector3 downInclineVector = Vector3.Cross(Vector3.Cross(Vector3.up, m_targetMoveDirection), m_groundHit.normal);
        Vector3 downSurfaceVector = Vector3.Cross(Vector3.Cross(Vector3.up, m_groundHit.normal), m_groundHit.normal);

        Gizmos.DrawRay(transform.position, downSurfaceVector * 100);

        //Debug TARGET direction
        Gizmos.color = Color.white;
        Gizmos.DrawLine(transform.position, transform.position + (m_targetMoveDirection * 3));

        //Debug MOVE direction
        Gizmos.color = Color.red;
        if (m_rb)
            Gizmos.DrawLine(transform.position, transform.position + new Vector3(m_rb.velocity.x, 0, m_rb.velocity.z));

        //Debug OUR direction
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + (transform.forward * 5));

        //Debug GROUNDED CHECKS
        Gizmos.color = Color.yellow;
        if (m_isGrounded)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position + new Vector3(0, m_groundedCheckRadius * 1.1f + 0.5f, 0) - new Vector3(0, stepHeight + 0.5f, 0) + (m_targetMoveDirection * stepHeight), m_groundedCheckRadius * 1.1f);
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position + new Vector3(0, m_groundedCheckRadius * 1.1f + 0.5f, 0) - new Vector3(0, stepHeight + 0.5f, 0), m_groundedCheckRadius * 1.1f);
            Gizmos.DrawRay(transform.position + Vector3.up, Vector3.down * 10); 
        }
        else
            Gizmos.DrawSphere(transform.position + new Vector3(0, m_groundedCheckRadius, 0) - new Vector3(0, m_groundedCheckRadius + Mathf.Abs(GetComponent<Rigidbody>().velocity.y * Time.fixedDeltaTime), 0), m_groundedCheckRadius * 0.9f);

        //Debug LAST GROUNDED
        if (m_isGrounded)
            Gizmos.color = Color.grey;
        else
            Gizmos.color = Color.black;
        //Gizmos.DrawLine(transform.position, groundedHit.point);
        Gizmos.DrawRay(m_groundHit.point, m_groundHit.normal * 10);

        Gizmos.color = Color.magenta;
        Gizmos.DrawWireSphere(transform.position + Vector3.up * (stepHeight + stepDistance), stepDistance);
        Gizmos.DrawWireSphere(transform.position, stepDistance);

        for(int i = 0; i < stepHits.Count; i++)
        {
            if (0 == i)
            {
                Gizmos.color = Color.yellow;

            }
            else
            {
                Gizmos.color = Color.magenta;

            }
            Gizmos.DrawSphere(stepHits[i].point, 0.1f);
            Gizmos.DrawRay(stepHits[i].point, stepHits[i].normal);
        }

        Gizmos.color = Color.black;
        Gizmos.DrawCube(m_groundHit.point, Vector3.one * 0.1f);


    }

    private void OnGUI()
    {
        //if (!Application.isEditor)
        //{
        //    return;
        //}
        //GUI.Label(new Rect(50, 50, 250, 50), "Grounded Status: " + (m_isGrounded ? (m_isSlipping ? "SLIPPING" : "GROUNDED") : "AIRTIME"));
        //
        //GUI.Label(new Rect(50,70, 250, 50), "Dist to ground: " + (Mathf.Round(m_distanceToGround *100)/100).ToString());
        ////
        //GUI.Label(new Rect(50, 90, 250, 50), "MoveAngle: " + (Mathf.Round(moveInclineAngle * 100) / 100).ToString());
        ////
        //GUI.Label(new Rect(50, 110, 250, 50), "SurfaceAngle: " + (Mathf.Round(downInclineAngle * 100) / 100).ToString());
        ////
        //GUI.Label(new Rect(50, 130, 250, 50), "Incline Angle Multiplier: " + (Mathf.Round(maxMoveSpeedOnCurrentIncline * 100) / 100).ToString());
        //
        //
        //GUI.Label(new Rect(50, 150, 250, 50), "TargetSpeed: " + (Mathf.Round(controlVariables.maxSpeed * 100) / 100).ToString());

    }
    #endregion


    private float GetStepHeight(float error = 0.05f)
    {
        stepHits = new List<RaycastHit>(Physics.SphereCastAll(transform.position + Vector3.up * (stepHeight + stepDistance), stepDistance, Vector3.down, (stepDistance + stepHeight), groundLayerMask, QueryTriggerInteraction.Ignore));
        RaycastHit targetStep = new RaycastHit();
        float targetStepClosestDistance = -1;

        for (int i = 0; i < stepHits.Count; i++)
        {
            RaycastHit newhit;
            Vector3 vectorTowardsHit = (new Vector3(stepHits[i].point.x, transform.position.y, stepHits[i].point.z) - transform.position).normalized;

            // + (vectorTowardsHit * error)
            if (Physics.SphereCast(stepHits[i].point + (Vector3.up * error * 2), error, Vector3.down, out newhit, error * 2, groundLayerMask, QueryTriggerInteraction.Ignore))
            {
                bool hitsAreDifferent = stepHits[i].collider != newhit.collider;
                bool stepBadIncline = Vector3.Angle(Vector3.up, newhit.normal) > m_maxMoveIncline;
                bool stepBelowRoot = stepHits[i].point.y < transform.position.y - error;
                bool stepAboveStepHeight = stepHits[i].point.y > transform.position.y + stepHeight;
                bool stepIsDirectlyBelow = Mathf.Approximately( stepHits[i].point.x, transform.position.x) && Mathf.Approximately(stepHits[i].point.z, transform.position.z);
                //Debug.Log(stepHits[i].collider.gameObject.name + "     NEWHITNAME: " + newhit.collider.gameObject.name + "  stepBadIncline: " + stepBadIncline + "   stepBelowRoot: " + stepBelowRoot + "   stepAboveStepHeight: " + stepAboveStepHeight + "   stepIsDirectlyBelow: " + stepIsDirectlyBelow);
                if (stepBadIncline || stepBelowRoot || stepAboveStepHeight || stepIsDirectlyBelow || hitsAreDifferent)
                {
                    stepHits.RemoveAt(i);
                    i--;
                    continue;
                }
                else
                {
                    newhit.point = stepHits[i].point;
                    stepHits[i] = newhit;
                    if (targetStepClosestDistance < 0)
                    {
                        targetStep = newhit;
                        targetStepClosestDistance = Vector3.Distance(transform.position - Vector3.up * m_distanceToGround, stepHits[i].point);
                    }
                    else
                    {
                        float distance = Vector2.Distance(transform.position - Vector3.up * m_distanceToGround, stepHits[i].point);
                        float height = stepHits[i].point.y - targetStep.point.y;
                        //determine the targetstep height using the distance & the height of the step
                        // || height > closestStepDistance
                        if (distance < targetStepClosestDistance)
                        {
                            targetStep = newhit;
                            targetStepClosestDistance = distance;
                        }
                    }
                }
            }
            else
            {
                //Debug.Log("Step Doesnt Exist");
                stepHits.RemoveAt(i);
                i--;
                continue;
            }


        }

        stepHits.Remove(targetStep);
        stepHits.Insert(0, targetStep);

        if (targetStepClosestDistance > 0)
        {
            float distanceBetweenStepAndCharacter = Vector2.Distance(new Vector2(transform.position.x, transform.position.z), new Vector2(targetStep.point.x, targetStep.point.z));
            //*Mathf.InverseLerp(stepDistance - m_groundedCheckRadius, m_groundedCheckRadius, distanceBetweenStepAndCharacter)
            return Mathf.Lerp(m_groundHit.point.y, targetStep.point.y, Mathf.InverseLerp(stepDistance, stepHeightRadiusOffset, distanceBetweenStepAndCharacter));

        }
        else
        {
            return  m_groundHit.point.y;
        }
    }

    
}
