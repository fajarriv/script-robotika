using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class MoveCar : MonoBehaviour
{
    [Header("Car Wheels (Wheel Collider)")]
    public WheelCollider FrontLeftWheel;
    public WheelCollider FrontRightWheel;
    public WheelCollider BackLeftWheel;
    public WheelCollider BackRightWheel;

    [Header("Car Spec")]
    public float turnSpeed = 7f;
    public float currentSpeed;
    private float maxSteerAngle = 40;
    public float maxSpeed = 1300f;
    public Vector3 centerOfMass;

    public AiSensor pathSensor;

    [Header("Obstacle Sensors config")]
    public float distance = 15;
    public float sideDistance = 5;
    public float angle = 20;
    public float height = 1;
    public Color meshColor = Color.blue;
    //public int scanFrequency = 30;
    public float sensorLength = 15f;
    public Vector3 frontSensorPosition = new Vector3(0f, 0.8f, 1.7f);
    public float frontSideSensorPositionObs = 1f;
    //public float frontSensorAngle = 35;
    public float frontSensorAngleObs = 60;
    public float steep = 6.35f;
    public float sideSteep = 41f;

    [Header("SteerControl")]
    public bool isAvoiding = false;
    public float avoidMultiplier = 0f;
    public float targetSteerAngle = 0;

    [Header("Path")]
    public List<Transform> pathNodes;
    public int currentPath = 0;
    public GameObject temporary;
    public GameObject temporary2;

    // Mesh sensors
    readonly Collider[] colliders = new Collider[1000];
    Mesh mesh;
    Mesh sideMesh;


    void Start()
    {
        //scanInterval = 1.0f / scanFrequency;
        GetComponent<Rigidbody>().centerOfMass = centerOfMass;
        pathSensor = GetComponent<AiSensor>();

        // find furthest path
        GameObject path = FindPath();
        pathNodes = new List<Transform>();

        if (transform != path.transform)
        {
            pathNodes.Add(path.transform);
        }
    }

    void Update()
    {
        if (pathNodes.Count > 0)
        {
            CheckWaypointDistance();
        }
        MoveWheels();
        // Finding next path
        GameObject path = FindPath();
        if ((path != null) && (!pathNodes.Contains(path.transform)))
        {
            pathNodes.Add(path.transform);
            // Add temporary variable if the next path is the U turn
            if (path.name.Equals("Track_Corner_90d_type_01_15x15m_free_obs (1)") && temporary != null)
            {
                pathNodes.Add(temporary.transform);
                temporary = null;
            }
        }
        RunSteer();
        SensorObs();
        if(temporary2 == null)
        {
            Brake();
        }
        LerpToSteerAngle();
    }

    private void CheckWaypointDistance()
    {
        if (Vector3.Distance(transform.position, pathNodes[currentPath].position) < 10f)
        {
            // Check if the currentPath is the last available pathnodes
            if (currentPath == pathNodes.Count - 1)
            {
                //currentPath = 0;
            }
            else
            {
                currentPath++;
            }
        }
    }

    private void MoveWheels()
    {
        currentSpeed = 2 * Mathf.PI * BackLeftWheel.radius * BackLeftWheel.rpm * 60 / 1000;

        if (currentSpeed < maxSpeed)
        {
            BackLeftWheel.motorTorque = 100f;
            BackRightWheel.motorTorque = 100f;
        }
        else
        {
            BackLeftWheel.motorTorque = 0;
            BackRightWheel.motorTorque = 0;
        }
    }

    private void RunSteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(pathNodes[currentPath].transform.position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
        targetSteerAngle = newSteer;

    }

    private void SensorObs()
    {
        avoidMultiplier = 0;
        isAvoiding = false;
        RaycastHit hit;

        Vector3 originFront = transform.position;
        originFront += transform.forward * frontSensorPosition.z;
        originFront += transform.up * frontSensorPosition.y;

        // FRONT SENSOR
        // center front hit
        if (Physics.Raycast(originFront, transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(originFront, hit.point);
            if (hit.collider.CompareTag("Terrain"))
            {
                isAvoiding = true;
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (19)"))
                {
                    Debug.DrawLine(originFront, hit.point);
                    avoidMultiplier = -1.125f;
                }
            }
        }

        // SIDE RIGHT SENSOR
        // Check if it hits Right side area
        originFront += transform.right * frontSideSensorPositionObs;
        if (Physics.Raycast(originFront, transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(originFront, hit.point);
            if (hit.collider.CompareTag("Terrain"))
            {
                isAvoiding = true;
                avoidMultiplier -= 0.6f;
                // stuck hitbox 1
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (19)"))
                {
                    Debug.DrawLine(originFront, hit.point);
                    avoidMultiplier = -1.125f;
                }
            }
        }
        // Check if it hits right side in angle
        else if (Physics.Raycast(originFront, Quaternion.AngleAxis(frontSensorAngleObs, transform.up) * transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(originFront, hit.point);
            isAvoiding = true;
            avoidMultiplier -= 0.4f;
            // stuck hitbox 1
            if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
            {
                avoidMultiplier = 1.1f;
            }
            if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (19)"))
            {
                Debug.DrawLine(originFront, hit.point);
                avoidMultiplier = -1.125f;
            }

        }

        // SIDE LEFT SENSOR
        // Check if it hits Left side area
        originFront -= transform.right * frontSideSensorPositionObs * 2;
        if (Physics.Raycast(originFront, transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(originFront, hit.point);

            if (hit.collider.CompareTag("Terrain"))
            {
                isAvoiding = true;
                avoidMultiplier += 0.6f;
                // stuck hitbox 1
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (19)"))
                {
                    Debug.DrawLine(originFront, hit.point);
                    avoidMultiplier = -1.125f;
                }
            }
        }
        // Check if it hits Left side in angle
        else if (Physics.Raycast(originFront, Quaternion.AngleAxis(-frontSensorAngleObs, transform.up) * transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(originFront, hit.point);
            isAvoiding = true;
            avoidMultiplier += 0.4f;
            // stuck hitbox 1
            if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
            {
                avoidMultiplier = 1.1f;
            }
            if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (19)"))
            {
                Debug.DrawLine(originFront, hit.point);
                avoidMultiplier = -1.125f;
            }
        }

        // Handle for front center hit
        //front center
        if (avoidMultiplier == 0)
        {
            if (Physics.Raycast(originFront, transform.forward, out hit, sensorLength))
            {
                if (hit.collider.CompareTag("Terrain") && !hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    Debug.DrawLine(originFront, hit.point);
                    isAvoiding = true;

                    if (hit.normal.x < 0)
                    {
                        avoidMultiplier = -0.7f;
                    }
                    else
                    {
                        avoidMultiplier = 0.7f;
                    }
                }
                // hitbox 1 stuck
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (19)"))
                {
                    Debug.DrawLine(originFront, hit.point);
                    avoidMultiplier = -1.125f;
                }

            }
        }



        // Set steer angle based on avoid multiplier
        if (isAvoiding)
        {
            targetSteerAngle = maxSteerAngle * avoidMultiplier;
        }


    }

    // modify so the picked path is the furthest path
    GameObject FindPath()
    {
        string currentFurthestNode = pathNodes[pathNodes.Count - 1].name;

        if (pathSensor.Objects.Count > 0)
        {
            GameObject tmp = pathSensor.Objects[0];
            foreach (GameObject obj in pathSensor.Objects)
            {
                // Straight line after the corner
                if (!currentFurthestNode.Equals("Track_line_type_01_30m_free_obs"))
                {
                    if (Vector3.Distance(transform.position, obj.transform.position) >= Vector3.Distance(transform.position, tmp.transform.position))
                    {
                        tmp = obj;
                    }

                }
                else
                {
                    // condition to make sure not taking straight line again
                    if (!(obj.name.Equals("Track_line_type_01_30m_free_obs (1)") ||
                         obj.name.Equals("Track_line_type_01_30m_free_obs (2)") ||
                             obj.name.Equals("Track_line_type_01_30m_free_obs (3)")))
                    {
                        tmp = obj;
                    }
                    // Save straight line after the U turn
                    if (obj.name.Equals("Track_line_type_01_30m_free_obs (1)"))
                    {
                        temporary = obj;
                    }
                }
                if (obj.name.Equals("Track_line_type_01_30m_free_obs (9)"))
                {
                    temporary2 = obj;
                }
            }
            return tmp;
        }
        return null;
    }

    void Brake()
    {

        if (pathNodes[currentPath].name.Contains("Track_Corner"))
        {
            //maxSpeed = 1400;
            BackLeftWheel.brakeTorque = 28f;
            BackRightWheel.brakeTorque = 28f;
            FrontLeftWheel.brakeTorque = 28f;
            FrontRightWheel.brakeTorque = 28f;

            //BackLeftWheel.motorTorque = 40;
            //BackRightWheel.motorTorque = 40;
        }
        else
        {
            //  maxSpeed = 1500;
            BackLeftWheel.brakeTorque = 0f;
            BackRightWheel.brakeTorque = 0f;
            FrontLeftWheel.brakeTorque = 0f;
            FrontRightWheel.brakeTorque = 0f;
            //          BackLeftWheel.motorTorque = -5f;
            //BackRightWheel.motorTorque = -5f;

        }

        //reverse torque when there is obstacle
        if (avoidMultiplier != 0)
        {
            BackLeftWheel.motorTorque = -3f;
            BackRightWheel.motorTorque = -3f;
        }

    }

    void LerpToSteerAngle()
    {

        if (pathNodes[currentPath].name.Equals("Track_Corner_90d_type_01_30x30m_free_obs"))
        {
            FrontLeftWheel.steerAngle = Mathf.Lerp(FrontLeftWheel.steerAngle, targetSteerAngle, Time.deltaTime * 0.1f);
            FrontRightWheel.steerAngle = Mathf.Lerp(FrontRightWheel.steerAngle, targetSteerAngle, Time.deltaTime * 0.1f);
        }
        else
        {
            FrontLeftWheel.steerAngle = Mathf.Lerp(FrontLeftWheel.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
            FrontRightWheel.steerAngle = Mathf.Lerp(FrontRightWheel.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
        }
    }


    Mesh CreateWedgeMesh()
    {
        Mesh frontMesh = new Mesh();

        int segments = 10;
        int numTriangles = (segments * 4) + 2 + 2;
        int numVertices = numTriangles * 3;

        Vector3[] vertices = new Vector3[numVertices];
        int[] triangles = new int[numVertices];

        Vector3 bottomCenter = Vector3.down * 1;
        Vector3 bottomLeft = Quaternion.Euler(steep, -angle, 0) * Vector3.forward * distance;
        Vector3 bottomRight = Quaternion.Euler(steep, angle, 0) * Vector3.forward * distance;

        Vector3 topCenter = bottomCenter + Vector3.up * height;
        Vector3 topLeft = bottomLeft + Vector3.up * height;
        Vector3 topRight = bottomRight + Vector3.up * height;
        int vert = 0;

        // left side
        vertices[vert++] = bottomCenter;
        vertices[vert++] = bottomLeft;
        vertices[vert++] = topLeft;

        vertices[vert++] = topLeft;
        vertices[vert++] = topCenter;
        vertices[vert++] = bottomCenter;

        // right side
        vertices[vert++] = bottomCenter;
        vertices[vert++] = topCenter;
        vertices[vert++] = topRight;

        vertices[vert++] = topRight;
        vertices[vert++] = bottomRight;
        vertices[vert++] = bottomCenter;

        float currentAngle = -angle;
        float deltaAngle = (angle * 2) / segments;

        for (int i = 0; i < segments; ++i)
        {
            bottomLeft = Quaternion.Euler(steep, currentAngle, 0) * Vector3.forward * distance;
            bottomRight = Quaternion.Euler(steep, currentAngle + deltaAngle, 0) * Vector3.forward * distance;

            topLeft = bottomLeft + Vector3.up * height;
            topRight = bottomRight + Vector3.up * height;

            // far side
            vertices[vert++] = bottomLeft;
            vertices[vert++] = bottomRight;
            vertices[vert++] = topRight;

            vertices[vert++] = topRight;
            vertices[vert++] = topLeft;
            vertices[vert++] = bottomLeft;

            // top side
            vertices[vert++] = topCenter;
            vertices[vert++] = topLeft;
            vertices[vert++] = topRight;

            // bottom side
            vertices[vert++] = bottomCenter;
            vertices[vert++] = bottomRight;
            vertices[vert++] = bottomLeft;

            currentAngle += deltaAngle;
        }

        for (int i = 0; i < numVertices; ++i)
        {
            triangles[i] = i;
        }

        frontMesh.vertices = vertices;
        frontMesh.triangles = triangles;
        frontMesh.RecalculateNormals();

        return frontMesh;
    }

    Mesh CreateSideWedgeMesh()
    {
        Mesh sideMesh = new Mesh();

        int segments = 10;
        int numTriangles = ((segments * 4) + 2 + 2) * 2;
        int numVertices = numTriangles * 3;

        Vector3[] vertices = new Vector3[numVertices];
        int[] triangles = new int[numVertices];


        // RIGHT SIDE SENSOR
        Vector3 bottomCenterR = Vector3.down * 1 + Vector3.right * frontSideSensorPositionObs;
        Vector3 bottomLeftR = Quaternion.Euler(sideSteep, -angle, 0) * (Quaternion.AngleAxis(frontSensorAngleObs, transform.up) * transform.forward) * sideDistance + Vector3.right * frontSideSensorPositionObs;
        Vector3 bottomRightR = Quaternion.Euler(sideSteep, angle, 0) * (Quaternion.AngleAxis(frontSensorAngleObs, transform.up) * transform.forward) * sideDistance + Vector3.right * frontSideSensorPositionObs;

        Vector3 topCenterR = bottomCenterR + Vector3.up * height;
        Vector3 topLeftR = bottomLeftR + Vector3.up * height;
        Vector3 topRightR = bottomRightR + Vector3.up * height;
        int vert = 0;

        // left side
        vertices[vert++] = bottomCenterR;
        vertices[vert++] = bottomLeftR;
        vertices[vert++] = topLeftR;

        vertices[vert++] = topLeftR;
        vertices[vert++] = topCenterR;
        vertices[vert++] = bottomCenterR;

        // right side
        vertices[vert++] = bottomCenterR;
        vertices[vert++] = topCenterR;
        vertices[vert++] = topRightR;

        vertices[vert++] = topRightR;
        vertices[vert++] = bottomRightR;
        vertices[vert++] = bottomCenterR;

        float currentAngleR = -angle;
        float deltaAngle = (angle * 2) / segments;

        for (int i = 0; i < segments; ++i)
        {
            bottomLeftR = Quaternion.Euler(sideSteep, currentAngleR, 0) * (Quaternion.AngleAxis(frontSensorAngleObs, transform.up) * transform.forward) * sideDistance + Vector3.right * frontSideSensorPositionObs;
            bottomRightR = Quaternion.Euler(sideSteep, currentAngleR + deltaAngle, 0) * (Quaternion.AngleAxis(frontSensorAngleObs, transform.up) * transform.forward) * sideDistance + Vector3.right * frontSideSensorPositionObs;

            topLeftR = bottomLeftR + Vector3.up * height;
            topRightR = bottomRightR + Vector3.up * height;

            // far side
            vertices[vert++] = bottomLeftR;
            vertices[vert++] = bottomRightR;
            vertices[vert++] = topRightR;

            vertices[vert++] = topRightR;
            vertices[vert++] = topLeftR;
            vertices[vert++] = bottomLeftR;

            // top side
            vertices[vert++] = topCenterR;
            vertices[vert++] = topLeftR;
            vertices[vert++] = topRightR;

            // bottom side
            vertices[vert++] = bottomCenterR;
            vertices[vert++] = bottomRightR;
            vertices[vert++] = bottomLeftR;

            currentAngleR += deltaAngle;
        }


        // LEFT SIDE SENSOR
        Vector3 bottomCenterL = Vector3.down * 1 - Vector3.right * frontSideSensorPositionObs;
        Vector3 bottomLeftL = Quaternion.Euler(sideSteep, -angle, 0) * (Quaternion.AngleAxis(-frontSensorAngleObs, transform.up) * transform.forward) * sideDistance - Vector3.right * frontSideSensorPositionObs;
        Vector3 bottomRightL = Quaternion.Euler(sideSteep, angle, 0) * (Quaternion.AngleAxis(-frontSensorAngleObs, transform.up) * transform.forward) * sideDistance - Vector3.right * frontSideSensorPositionObs;


        Vector3 topCenterL = bottomCenterL + Vector3.up * height;
        Vector3 topLeftL = bottomLeftL + Vector3.up * height;
        Vector3 topRightL = bottomRightL + Vector3.up * height;

        // left side
        vertices[vert++] = bottomCenterL;
        vertices[vert++] = bottomLeftL;
        vertices[vert++] = topLeftL;

        vertices[vert++] = topLeftL;
        vertices[vert++] = topCenterL;
        vertices[vert++] = bottomCenterL;

        // right side
        vertices[vert++] = bottomCenterL;
        vertices[vert++] = topCenterL;
        vertices[vert++] = topRightL;

        vertices[vert++] = topRightL;
        vertices[vert++] = bottomRightL;
        vertices[vert++] = bottomCenterL;

        float currentAngleL = -angle;
        for (int i = 0; i < segments; ++i)
        {
            bottomLeftL = Quaternion.Euler(sideSteep, currentAngleL, 0) * (Quaternion.AngleAxis(-frontSensorAngleObs, transform.up) * transform.forward) * sideDistance - Vector3.right * frontSideSensorPositionObs;
            bottomRightL = Quaternion.Euler(sideSteep, currentAngleL + deltaAngle, 0) * (Quaternion.AngleAxis(-frontSensorAngleObs, transform.up) * transform.forward) * sideDistance - Vector3.right * frontSideSensorPositionObs;

            topLeftL = bottomLeftL + Vector3.up * height;
            topRightL = bottomRightL + Vector3.up * height;

            // far side
            vertices[vert++] = bottomLeftL;
            vertices[vert++] = bottomRightL;
            vertices[vert++] = topRightL;

            vertices[vert++] = topRightL;
            vertices[vert++] = topLeftL;
            vertices[vert++] = bottomLeftL;

            // top side
            vertices[vert++] = topCenterL;
            vertices[vert++] = topLeftL;
            vertices[vert++] = topRightL;

            // bottom side
            vertices[vert++] = bottomCenterL;
            vertices[vert++] = bottomRightL;
            vertices[vert++] = bottomLeftL;

            currentAngleL += deltaAngle;
        }

        for (int i = 0; i < numVertices; ++i)
        {
            triangles[i] = i;
        }

        sideMesh.vertices = vertices;
        sideMesh.triangles = triangles;
        sideMesh.RecalculateNormals();

        return sideMesh;
    }

    private void OnValidate()
    {
        mesh = CreateWedgeMesh();
        sideMesh = CreateSideWedgeMesh();
        //scanInterval = 1.0f / scanFrequency;
    }

    private void OnDrawGizmos()
    {
        if (mesh && sideMesh)
        {
            Gizmos.color = meshColor;
            Gizmos.DrawMesh(mesh, transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, transform.rotation);
            Gizmos.DrawMesh(sideMesh, transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, transform.rotation);
        }

    }



}