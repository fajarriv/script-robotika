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

    [Header("Sensors config")]
    public AiSensor pathSensor;
    public float sensorLength = 9f;
    public Vector3 frontSensorPosition = new Vector3(0f, 0.8f, 1.7f);
    public float frontSideSensorPosition = 1f;
    public float frontSensorAngle = 35;
    public bool isAvoiding = false;
    public float avoidMultiplier = 0f;
    private float targetSteerAngle = 0;

    public List<Transform> pathNodes;
    private int currentPath = 0;

    void Start()
    {
        GetComponent<Rigidbody>().centerOfMass = centerOfMass;
        pathSensor = GetComponent<AiSensor>();

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
        GameObject path = FindPath();
        if (!pathNodes.Contains(path.transform))
        {
            pathNodes.Add(path.transform);
        }
        RunSteer();
        Sensor();
        LerpToSteerAngle();
    }

    private void CheckWaypointDistance()
    {
        if (Vector3.Distance(transform.position, pathNodes[currentPath].position) < 10f)
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
        isAvoiding = false;

        Debug.DrawRay(sensorStartPos, new Vector3(0,0,20f));
        // front
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                isAvoiding = true;
            }
        }

        // front right
        sensorStartPos += transform.right * frontSideSensorPosition;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                isAvoiding = true;
                avoidMultiplier -= 0.5f;
            }
        }

        // front right angle
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                isAvoiding = true;
                avoidMultiplier -= 0.2f;
            }
        }

        // front left 
        sensorStartPos -= transform.right * frontSideSensorPosition * 2;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                isAvoiding = true;
                avoidMultiplier += 0.5f;
            }
        }

        // front left angle
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                isAvoiding = true;
                avoidMultiplier += 0.2f;
            }
        }

        //front center
        if (avoidMultiplier == 0)
        {
            if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
            {
                if (hit.collider.CompareTag("Terrain"))
                {
                    Debug.DrawLine(sensorStartPos, hit.point);
                    isAvoiding = true;

                    if (hit.normal.x < 0)
                    {
                        //    avoidMultiplier = -0.7f;
                        //}
                        //else
                        //{
                        avoidMultiplier = 1f;
                    }
                }
            }
        }

        if (isAvoiding)
        {
            targetSteerAngle = maxSteerAngle * avoidMultiplier;
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
        Vector3 relativeVector = transform.InverseTransformPoint(pathNodes[currentPath].position);
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
                    tmp = obj;
                }
            }

            return tmp;
        }
        return null;
    }

    void LerpToSteerAngle()
    {
        FrontLeftWheel.steerAngle = Mathf.Lerp(FrontLeftWheel.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
        FrontRightWheel.steerAngle = Mathf.Lerp(FrontRightWheel.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
    }
}