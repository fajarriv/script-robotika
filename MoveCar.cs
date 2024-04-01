﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class MoveCar : MonoBehaviour
{
    public WheelCollider LeftWheel;
    public WheelCollider RightWheel;
    public float SteerAngle;
    public AiSensor pathSensor;
    public float turnSpeed = 5f;
    public float currentSpeed;
    private float maxSteerAngle = 40;
    public float maxSpeed = 1500f;

    [Header("Sensors")]
    public float sensorLength = 3f;
    public Vector3 frontSensorPosition = new Vector3(0f, 0.2f, 0.5f);
    public float frontSideSensorPosition = 0.2f;
    public float frontSensorAngle = 30;
    public bool avoiding = false;
    public float avoidMultiplier = 0f;
    private float targetSteerAngle = 0;

    public List<Transform> pathNodes;
    private int currentPath = 0;
    public float steerAngle = 0;

    void Start()
    {
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
        avoiding = false;

        // front
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
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
            }
        }

        // front left 
        sensorStartPos -= transform.right * frontSideSensorPosition * 2;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
                avoidMultiplier += 0.5f;
            }
        }
        
        // front left angle
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("Terrain"))
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                avoiding = true;
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
                    avoiding = true;

                    if (hit.normal.x < 0)
                    {
                        avoidMultiplier = -0.8f;
                    }
                    else
                    {
                        avoidMultiplier = 0.8f;
                    }
                }
            }
        }

        if (avoiding)
        {
            targetSteerAngle = maxSteerAngle * avoidMultiplier;
            steerAngle = maxSteerAngle * avoidMultiplier;
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
        Vector3 relativeVector = transform.InverseTransformPoint(pathNodes[currentPath].position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
        targetSteerAngle = newSteer;

        steerAngle = newSteer;
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
        LeftWheel.steerAngle = Mathf.Lerp(LeftWheel.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
        RightWheel.steerAngle = Mathf.Lerp(RightWheel.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
    }
}