using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class MoveCar : MonoBehaviour
{
    //public Transform path;

    //private List<Transform> nodes;
    //private int currentNode = 0;

    public WheelCollider LeftWheel;
    public WheelCollider RightWheel;
    public float SteerAngle;
    public AiSensor pathSensor;
    public GameObject path;

   private float maxSteerAngle = 40f;


    void Start()
    {
        pathSensor = GetComponent<AiSensor>();
        //Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();

        //nodes = new List<Transform>();

        //for (int i = 0; i < pathTransforms.Length; i++)
        //{
        //    if (pathTransforms[i] != path.transform)
        //    {
        //        nodes.Add(pathTransforms[i]);
        //    }
        //}
        path = FindPath();
    }

    void Update()
    {
        //CheckWaypointDistance();
        MoveWheels();
        path = FindPath();

        // check if obstacle exist
        if (pathSensor.obstacle != null)
        {
            AvoidObstacle(pathSensor.obstacle);
        }
        else
        {
            RunSteer(path);

        }


        if (path)
        {
        }
        //Sensors();
    }

    private void CheckObstacle()
    {
        if (pathSensor.obstacle)
        {

        }
    }


    private void AvoidObstacle(GameObject obstacle)
    {
        Vector3 obstacelRelativeVector = transform.InverseTransformPoint(obstacle.transform.position);

        float steerConstant; 

        if(obstacelRelativeVector.z >= 0)
        {
            steerConstant = 0.2f;
        }
        else
        {
            steerConstant = -0.2f;
        }


        if (Vector3.Distance(transform.position, obstacle.transform.position)< 10f) {
            float newSteer = ((obstacelRelativeVector.x  + steerConstant )/ obstacelRelativeVector.magnitude) * maxSteerAngle;

            LeftWheel.steerAngle = newSteer;
            RightWheel.steerAngle = newSteer;
            SteerAngle = newSteer;

        }
    }

    private void MoveWheels()
    {
        LeftWheel.motorTorque = 50f;
        RightWheel.motorTorque = 50f;
    }

    private void RunSteer(GameObject path)
    {
        Vector3 relativeVector = transform.InverseTransformPoint(path.transform.position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;

        LeftWheel.steerAngle = newSteer;
        RightWheel.steerAngle = newSteer;
        SteerAngle = newSteer;
    }

    //private void CheckWaypointDistance()
    //{
    //    if (Vector3.Distance(transform.position, nodes[currentNode].position) < 10f)
    //    {
    //        if (currentNode == nodes.Count - 1)
    //        {
    //            currentNode = 0;
    //        }
    //        else
    //        {
    //            currentNode++;
    //        }
    //    }
    //}

    //private void Sensors()
    //{
    //    RaycastHit hit;
    //    float sensorLength = 30f;

    //    if (Physics.Raycast(transform.position, transform.forward, out hit, sensorLength))
    //    {
    //        if (hit.collider.CompareTag("Terrain"))
    //        {
    //            Debug.DrawLine(transform.position, hit.point, Color.red);
    //        }
    //    }
    //}


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
}
