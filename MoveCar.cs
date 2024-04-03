using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class MoveCar : MonoBehaviour
{
    public WheelCollider LeftWheel;
    public WheelCollider RightWheel;
    public AiSensor pathSensor;
    public float turnSpeed = 7f;
    public float currentSpeed;
    private float maxSteerAngle = 40;
    public float maxSpeed = 1300f;
    public Vector3 centerOfMass;

    [Header("Sensors")]
    public float sensorLength = 9f;
    public Vector3 frontSensorPosition = new Vector3(0f, 0.8f, 1.7f);
    public float frontSideSensorPosition = 1f;
    public float frontSensorAngle = 35;
    public bool avoiding = false;
    public float avoidMultiplier = 0f;
    private float targetSteerAngle = 0;

    public List<GameObject> pathNodes;
    private int currentPath = 0;

    [Header("Obstacle Sensor")]
    public float distance = 9;
    public float sideDistance = 3;
    public float angle = 12;
    public float height = 1.5f;
    public Color meshColor = Color.blue;
    public int scanFrequency = 30;
    public LayerMask layers;
    public List<GameObject> Objects = new List<GameObject>();
    public float frontSideSensorPositionObs = 1f;
    public float frontSensorAngleObs = 60;
    public float steep = 6.35f;
    public float sideSteep = 41f;

    Collider[] colliders = new Collider[1000];
    Mesh mesh;
    Mesh sideMesh;
    int count;
    float scanInterval;
    float scanTimer;

    void Start()
    {
        scanInterval = 1.0f / scanFrequency;
        GetComponent<Rigidbody>().centerOfMass = centerOfMass;
        pathSensor = GetComponent<AiSensor>();

        GameObject path = FindPath();
        pathNodes = new List<GameObject>();

        if (transform != path.transform)
        {
            pathNodes.Add(path);
        }
    }

    void Update()
    {
        if (pathNodes.Count > 0)
        {
            CheckWaypointDistance();
        }
        MoveWheels();
        GameObject path = FindPath();
        if (!pathNodes.Contains(path))
        {
            pathNodes.Add(path);
        }
        RunSteer();
        Sensor();
        scanTimer -= Time.fixedDeltaTime;
        if (scanTimer < 0)
        {
            scanTimer += scanInterval;
            Scan();
        }
        //Sensor();
        LerpToSteerAngle();
    }

    private void CheckWaypointDistance()
    {
        if (Vector3.Distance(transform.position, pathNodes[currentPath].transform.position) < 10f)
        {
            if (currentPath == pathNodes.Count - 1)
            {
                currentPath = 0;
            }
            else
            {
                currentPath++;
            }
        }
    }

    private void Sensor()
    {
        RaycastHit hit;
        Vector3 sensorStartPos = transform.position;
        sensorStartPos += transform.forward * frontSensorPosition.z;
        sensorStartPos += transform.up * frontSensorPosition.y;
        avoidMultiplier = 0;
        avoiding = false;

        // front
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
            }
        }

        // front right
        sensorStartPos += transform.right * frontSideSensorPosition;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier -= 0.5f;
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
            }
        }

        // front right angle
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier -= 0.2f;
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
            }
        }

        // front left 
        sensorStartPos -= transform.right * frontSideSensorPosition * 2;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain") && !hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier += 0.5f;
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
            }
        }

        // front left angle
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain") && !hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier += 0.2f;
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    avoidMultiplier = 1.1f;
                }
            }

        }

        //front center
        if (avoidMultiplier == 0)
        {
            if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
            {
                if (hit.collider.CompareTag("Terrain") && !hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    Debug.DrawLine(sensorStartPos, hit.point);
                    avoiding = true;

                    if (hit.normal.x < 0)
                    {
                        avoidMultiplier = -0.7f;
                    }
                    else
                    {
                        avoidMultiplier = 0.7f;
                    }
                }
                if (hit.collider.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                {
                    Debug.DrawLine(sensorStartPos, hit.point);
                    avoidMultiplier = 1.1f;
                }
            }
        }

        if (avoiding)
        {
            targetSteerAngle = maxSteerAngle * avoidMultiplier;
        }
    }

    private void MoveWheels()
    {
        currentSpeed = 2 * Mathf.PI * LeftWheel.radius * LeftWheel.rpm * 60 / 1000;

        if (currentSpeed < maxSpeed)
        {
            LeftWheel.motorTorque = 100f;
            RightWheel.motorTorque = 100f;
        }
        else
        {
            LeftWheel.motorTorque = 0;
            RightWheel.motorTorque = 0;
        }
    }

    private void RunSteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(pathNodes[currentPath].transform.position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
        targetSteerAngle = newSteer;
    }

    // modify so the picked path is the furthest path
    GameObject FindPath()
    {
        if (pathSensor.Objects.Count > 0)
        {
            GameObject tmp = pathSensor.Objects[0];
            foreach (GameObject obj in pathSensor.Objects)
            {
                if (Vector3.Distance(transform.position, obj.transform.position) >= Vector3.Distance(transform.position, tmp.transform.position))
                {
                    if (!(pathNodes[pathNodes.Count - 1].name.Equals("Track_line_type_01_30m_free_obs") &
                        (obj.name.Equals("Track_line_type_01_30m_free_obs (1)") |
                        obj.name.Equals("Track_line_type_01_30m_free_obs (2)") |
                        obj.name.Equals("Track_line_type_01_30m_free_obs (3)"))))
                    {
                        tmp = obj;
                    }
                }
            }

            return tmp;
        }
        return null;
    }

    void LerpToSteerAngle()
    {
        if (pathNodes[currentPath].name.Equals("Track_Corner_90d_type_01_30x30m_free_obs"))
        {
            LeftWheel.steerAngle = Mathf.Lerp(LeftWheel.steerAngle, targetSteerAngle, Time.deltaTime * 0.22f);
            RightWheel.steerAngle = Mathf.Lerp(RightWheel.steerAngle, targetSteerAngle, Time.deltaTime * 0.22f);
        }
        else
        {
            LeftWheel.steerAngle = Mathf.Lerp(LeftWheel.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
            RightWheel.steerAngle = Mathf.Lerp(RightWheel.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
        }
    }
    private void Scan()
    {
        count = Physics.OverlapSphereNonAlloc(transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, distance, colliders, layers, QueryTriggerInteraction.Collide);

        Objects.Clear();
        for (int i = 0; i < count; ++i)
        {
            GameObject obj = colliders[i].gameObject;
            if (obj.CompareTag("Terrain") || obj.name.Contains("Track_Fence"))
            {
                IsInsight(obj);
            }
        }
    }

    public void IsInsight(GameObject obj)
    {
        // FRONT SENSOR
        Vector3 originFront = transform.position + transform.forward * frontSensorPosition.z + transform.up * frontSensorPosition.y;
        Vector3 destFront = obj.transform.position;
        Vector3 directionFront = destFront - originFront;

        // SIDE RIGHT SENSOR
        Vector3 originSideR = transform.position + Vector3.right * frontSideSensorPositionObs + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y;
        Vector3 destSideR = obj.transform.position;
        Vector3 directionSideR = destSideR - originSideR;

        // SIDE LEFT SENSOR
        Vector3 originSideL = transform.position - Vector3.right * frontSideSensorPositionObs + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y;
        Vector3 destSideL = obj.transform.position;
        Vector3 directionSideL = destSideL - originSideL;

        bool indicator = false;

        if (directionFront.y <= height && directionSideR.y <= height && directionSideL.y <= height)
        {

            directionFront.y = 0;
            float deltaAngleFront = Vector3.Angle(directionFront, transform.forward);

            directionSideR.y = 0;
            float deltaAngleSideR = Vector3.Angle(directionSideR, (Quaternion.AngleAxis(frontSensorAngleObs, transform.up) * transform.forward));

            directionSideL.y = 0;
            float deltaAngleSideL = Vector3.Angle(directionSideL, (Quaternion.AngleAxis(-frontSensorAngleObs, transform.up) * transform.forward));


            //// Kena sensor kanan
            //if (deltaAngleSideR <= angle)
            //{
            //    if (directionSideR.z <= sideDistance)
            //    {
            //        if (obj.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
            //        {
            //            avoidMultiplier = 1.1f;
            //        }
            //        targetSteerAngle = maxSteerAngle * avoidMultiplier;
            //        indicator = true;
            //    }
            //}

            // Kena sensor kiri
            if (deltaAngleSideL <= angle)
            {
                if (directionSideL.z <= sideDistance)
                {
                    if (obj.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                    {
                        avoidMultiplier = 1.1f;
                        avoiding = true;
                        targetSteerAngle = maxSteerAngle * avoidMultiplier;
                        indicator = true;
                    }
                }
            }

            // Kena sensor depanx
            if (deltaAngleFront <= angle)
            {
                if (directionFront.z <= distance)
                {
                    if (obj.name.Equals("Track_Fence_line_type_01_white_block_1&5m_free (10)"))
                    {
                        avoidMultiplier = 1.1f;
                        avoiding = true;
                        targetSteerAngle = maxSteerAngle * avoidMultiplier;
                        indicator = true;
                    }
                }
            }
           
        }

        if (indicator)
        {
            Objects.Add(obj);
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
        scanInterval = 1.0f / scanFrequency;
    }

    private void OnDrawGizmos()
    {
        if (mesh && sideMesh)
        {
            Gizmos.color = meshColor;
            Gizmos.DrawMesh(mesh, transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, transform.rotation);
            Gizmos.DrawMesh(sideMesh, transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, transform.rotation);
        }

        //Gizmos.DrawWireSphere(transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, distance);

        //Gizmos.DrawWireSphere(transform.position + Vector3.right * frontSideSensorPositionObs + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, sideDistance);
        //Gizmos.DrawWireSphere(transform.position - Vector3.right * frontSideSensorPositionObs + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, sideDistance);
        for (int i = 0; i < count; ++i)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(colliders[i].transform.position, 0.2f);
        }

        Gizmos.color = Color.green;
        foreach (var obj in Objects)
        {
            Gizmos.DrawSphere(obj.transform.position, 0.2f);
        }
    }

}