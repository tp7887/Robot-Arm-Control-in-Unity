using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class dis : MonoBehaviour
{
    public GameObject obj1;
    public GameObject obj2;
    // Start is called before the first frame update
    void Start()
    {
        float dist = Vector3.Distance(obj1.transform.position, obj2.transform.position);
        Debug.Log("Distance to other: " + dist);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
