using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections;
using System;
using Random = UnityEngine.Random;
using UnityEngine.InputSystem;

public class DoggyAgent : Agent
{
    [Header("Сервоприводы")]
    public ArticulationBody[] legs;

    [Header("Скорость работы сервоприводов")]
    public float servoSpeed;

    [Header("Тело")]
    public ArticulationBody body;
    private Vector3 defPos;
    private Quaternion defRot;
    public float strenghtMove;

    [Header("Куб (цель)")]
    public GameObject cube;

    [Header("Сенсоры")]
    public Unity.MLAgentsExamples.GroundContact[] groundContacts;

    private float distToTarget = 0f;

    //private Oscillator m_Oscillator;

    public override void Initialize()
    {
        distToTarget = Vector3.Distance(body.transform.position, cube.transform.position);
        defRot = body.transform.rotation;
        defPos = body.transform.position;

        //m_Oscillator = GetComponent<Oscillator>(); ***
        //m_Oscillator.ManagedReset(); ***
    }

    public void ResetDog()
    {
        Quaternion newRot = Quaternion.Euler(-90, 0, Random.Range(0f, 360f));


        body.TeleportRoot(defPos, newRot);
        //body.TeleportRoot(defPos, defRot); ***
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;

        for (int i = 0; i < 12; i++)
        {
            //MoveLeg(legs[i], Random.Range(legs[i].xDrive.lowerLimit, legs[i].xDrive.upperLimit));
            MoveLeg(legs[i], 0);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        Debug.Log("Heuristic");
    }

    public override void OnEpisodeBegin()
    {
        ResetDog();
        //m_Oscillator.ManagedReset(); ***

        //cube.transform.position = new Vector3(5, 0.21f, Random.Range(-2f, 2f));
        cube.transform.position = new Vector3(Random.Range(-7.5f, 7.5f), 0.21f, Random.Range(-7.5f, 7.5f));
        //cube.transform.position = new Vector3(5f, 0.21f, 0); ***

        //cube.transform.position = new Vector3(8f, 0.26f, 0f);
        distToTarget = Vector3.Distance(body.transform.position, cube.transform.position);

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(body.transform.position);
        sensor.AddObservation(body.velocity);
        sensor.AddObservation(body.angularVelocity);
        sensor.AddObservation(body.transform.right);

        // Позиция куба
        sensor.AddObservation(cube.transform.position);

        // Относительное положение куба
        Vector3 relativePosition = cube.transform.position - body.transform.position;
        sensor.AddObservation(relativePosition);

        // Угловая позиция куба
        Vector3 toCube = (cube.transform.position - body.transform.position).normalized;
        float angleToCube = Vector3.SignedAngle(body.transform.right, toCube, Vector3.up);
        sensor.AddObservation(angleToCube);

        // Расстояние до куба
        float distanceToCube = Vector3.Distance(body.transform.position, cube.transform.position);
        sensor.AddObservation(distanceToCube);
        foreach (var leg in legs)
        {
            sensor.AddObservation(leg.xDrive.target);
            sensor.AddObservation(leg.velocity);
            sensor.AddObservation(leg.angularVelocity);
        }

        foreach(var groundContact in groundContacts)
        {
            sensor.AddObservation(groundContact.touchingGround);
        }
    }

public override void OnActionReceived(ActionBuffers actionsBuffer)
{
    var actions = actionsBuffer.ContinuousActions;

    int i = 0;
    while (i < legs.Length)
    {
        var legDrive = legs[i].xDrive;
        float normalized = (actions[i] + 1f) / 2f;
        float angle = legDrive.lowerLimit + normalized * (legDrive.upperLimit - legDrive.lowerLimit);
        MoveLeg(legs[i], angle);
        i++;
    }

    Vector3 currentPos = body.transform.position;
    Vector3 goalPos = cube.transform.position;
    float currentDist = Vector3.Distance(currentPos, goalPos);

    float distanceReward = (distToTarget - currentDist) * 0.1f;
    AddReward(distanceReward);

    Vector3 directionToGoal = (goalPos - currentPos).normalized;
    float velocityReward = Vector3.Dot(body.velocity, directionToGoal) * 0.01f;
    AddReward(velocityReward);

    float uprightReward = Vector3.Dot(body.transform.up, Vector3.up) * 0.005f;
    AddReward(uprightReward);

    int groundContactCount = 0;
    foreach (var contact in groundContacts) {
        if (contact.touchingGround) groundContactCount++;
    }
    float contactReward = groundContactCount * 0.0005f;
    AddReward(contactReward);

    AddReward(-0.0005f);

    bool reachedGoal = currentDist <= 1.5f;
    if (reachedGoal)
    {
        AddReward(1f);
        EndEpisode();
    }

    distToTarget = currentDist;
}

    public void FixedUpdate()
    {
        body.AddForce((cube.transform.position - body.transform.position).normalized * strenghtMove);
        for (int i = 0; i < 12; i++)
        {
            legs[i].AddForce((cube.transform.position - body.transform.position).normalized * strenghtMove / 20f);
        }

        RaycastHit hit;
        if (Physics.Raycast(body.transform.position, body.transform.right, out hit))
        {
            if (hit.collider.gameObject == cube)
            {
                body.AddForce(2f * strenghtMove * (cube.transform.position - body.transform.position).normalized);
                for (int i = 0; i < 12; i++)
                {
                    legs[i].AddForce((cube.transform.position - body.transform.position).normalized * strenghtMove / 10f);
                }
            }
        }
        Debug.DrawRay(body.transform.position, body.transform.right, Color.white);
    }

    void MoveLeg(ArticulationBody leg, float targetAngle)
    {
        leg.GetComponent<Leg>().MoveLeg(targetAngle, servoSpeed);
    }
}
