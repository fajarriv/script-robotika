using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class AiSensor : MonoBehaviour
{
    public float distance = 35;
    public float angle = 40;
    public float height = 3f;
    public float steep = 2;
    public Color meshColor = Color.red;
    public int scanFrequency = 100;
    public LayerMask layers;
    public List<GameObject> Objects = new List<GameObject>();


    Collider[] colliders = new Collider[50];
    Mesh mesh;
    int count;
    float scanInterval;
    float scanTimer;
    Vector3 frontSensorPosition = new Vector3(0f, 0.8f, 1.7f);

    // corner handling
    //public List<GameObject> visitedCorner = new List<GameObject>();

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

        for (int i = 0; i < count; ++i)
        {
            GameObject obj = colliders[i].gameObject;

            if (IsInsight(obj) && obj.CompareTag("Untagged") && (obj.name.Contains("Track_line") || obj.name.Contains("Track_Corner")))
            {
                Objects.Add(obj);
            }

        }
    }

    public bool IsInsight(GameObject obj)
    {
        Vector3 origin = transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y;
        Vector3 dest = obj.transform.position;
        Vector3 direction = dest - origin;

        if (direction.y < -steep || direction.y > height)
        {
            return false;
        }

        direction.y = 0;
        float deltaAngle = Vector3.Angle(direction, transform.forward);

        if (deltaAngle > angle)
        {
            return false;
        }
        return true;
    }

    Mesh CreateWedgeMesh()
    {
        Mesh mesh = new Mesh();

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

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        return mesh;
    }

    private void OnValidate()
    {
        mesh = CreateWedgeMesh();
        scanInterval = 1.0f / scanFrequency;
    }

    private void OnDrawGizmos()
    {
        if (mesh)
        {
            Gizmos.color = meshColor;
            Gizmos.DrawMesh(mesh, transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, transform.rotation);
        }

        //Gizmos.DrawWireSphere(transform.position + transform.forward * frontSensorPosition.z + transform.transform.up * frontSensorPosition.y, distance);
        for (int i = 0; i < count; ++i)
        {
            Gizmos.DrawSphere(colliders[i].transform.position, 0.2f);
        }

        Gizmos.color = Color.green;
        foreach (var obj in Objects)
        {
            Gizmos.DrawSphere(obj.transform.position, 0.2f);
        }
    }

}