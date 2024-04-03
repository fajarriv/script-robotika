using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class ObstacleSensor : MonoBehaviour
{
    public float distance = 9;
    public float sideDistance = 3;
    public float angle = 12;
    public float height = 1.5f;
    public Color meshColor = Color.blue;
    public int scanFrequency = 30;
    public LayerMask layers;
    public List<GameObject> Objects = new List<GameObject>();
    public float frontSideSensorPosition = 1f;
    public float frontSensorAngle = 60;
    public float steep = 6.35f;
    public float sideSteep = 41f;

    Collider[] colliders = new Collider[50];
    Mesh mesh;
    Mesh sideMesh;
    int count;
    float scanInterval;
    float scanTimer;
    Vector3 frontSensorPosition = new Vector3(0f, 0.8f, 1.7f);

    void Start()
    {
        scanInterval = 1.0f / scanFrequency;
    }

    void Update()
    {
        scanTimer -= Time.fixedDeltaTime;
        if (scanTimer < 0)
        {
            scanTimer += scanInterval;
            Scan();
        }
    }

    private void Scan()
    {
        count = Physics.OverlapSphereNonAlloc(transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, distance, colliders, layers, QueryTriggerInteraction.Collide);

        Objects.Clear();
        //obstacle = null;
        for (int i = 0; i < count; ++i)
        {
            GameObject obj = colliders[i].gameObject;
            if (IsInsight(obj) && (obj.CompareTag("Terrain") || obj.name.Contains("Track_Fence")))
            {
                Objects.Add(obj);
            }
        }
    }

    public bool IsInsight(GameObject obj)
    {
        // FRONT SENSOR
        Vector3 originFront = transform.position + transform.forward * frontSensorPosition.z + transform.up * frontSensorPosition.y;
        Vector3 destFront = obj.transform.position;
        Vector3 directionFront = destFront - originFront;

        // SIDE RIGHT SENSOR
        Vector3 originSideR = transform.position + Vector3.right * frontSideSensorPosition + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y;
        Vector3 destSideR = obj.transform.position;
        Vector3 directionSideR = destSideR - originSideR;

        // SIDE LEFT SENSOR
        Vector3 originSideL = transform.position - Vector3.right * frontSideSensorPosition + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y;
        Vector3 destSideL = obj.transform.position;
        Vector3 directionSideL = destSideL - originSideL;

        if (directionFront.y > height && directionSideR.y > height && directionSideL.y > height)
        {
            return false;
        }

        directionFront.y = 0;
        float deltaAngleFront = Vector3.Angle(directionFront, transform.forward);

        directionSideR.y = 0;
        float deltaAngleSideR = Vector3.Angle(directionSideR, (Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward));

        directionSideL.y = 0;
        float deltaAngleSideL = Vector3.Angle(directionSideL, (Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward));

        if (deltaAngleFront > angle && deltaAngleSideR > angle && deltaAngleSideL > angle)
        {
            return false;
        }

        return true;
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
        Vector3 bottomCenterR = Vector3.down * 1 + Vector3.right * frontSideSensorPosition;
        Vector3 bottomLeftR = Quaternion.Euler(sideSteep, -angle, 0) * (Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward) * sideDistance + Vector3.right * frontSideSensorPosition;
        Vector3 bottomRightR = Quaternion.Euler(sideSteep, angle, 0) * (Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward) * sideDistance + Vector3.right * frontSideSensorPosition;

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
            bottomLeftR = Quaternion.Euler(sideSteep, currentAngleR, 0) * (Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward) * sideDistance + Vector3.right * frontSideSensorPosition;
            bottomRightR = Quaternion.Euler(sideSteep, currentAngleR + deltaAngle, 0) * (Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward) * sideDistance + Vector3.right * frontSideSensorPosition;

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
        Vector3 bottomCenterL = Vector3.down * 1 - Vector3.right * frontSideSensorPosition;
        Vector3 bottomLeftL = Quaternion.Euler(sideSteep, -angle, 0) * (Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward) * sideDistance - Vector3.right * frontSideSensorPosition;
        Vector3 bottomRightL = Quaternion.Euler(sideSteep, angle, 0) * (Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward) * sideDistance - Vector3.right * frontSideSensorPosition;


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
            bottomLeftL = Quaternion.Euler(sideSteep, currentAngleL, 0) * (Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward) * sideDistance - Vector3.right * frontSideSensorPosition;
            bottomRightL = Quaternion.Euler(sideSteep, currentAngleL + deltaAngle, 0) * (Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward) * sideDistance - Vector3.right * frontSideSensorPosition;

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

        //Gizmos.DrawWireSphere(transform.position + Vector3.right * frontSideSensorPosition + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, sideDistance);
        //Gizmos.DrawWireSphere(transform.position - Vector3.right * frontSideSensorPosition + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, sideDistance);
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