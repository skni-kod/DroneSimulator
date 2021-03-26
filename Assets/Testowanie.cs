using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Testowanie : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    private void FixedUpdate()
    {
        var rb = GetComponent<Rigidbody>();
        rb.AddRelativeTorque(Vector3.up * 1);
    }
}
