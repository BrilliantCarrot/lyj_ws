using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AgentCamera : MonoBehaviour
{
    public GameObject player;
    Vector3 offsetPos = new Vector3(0,5,-5);
    Vector3 offsetRot = new Vector3(45,0,0);

    void Update()
    {
        transform.position = player.transform.position + offsetPos;
        transform.rotation = Quaternion.Euler(offsetRot);
    }
}
