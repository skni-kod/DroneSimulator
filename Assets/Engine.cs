using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Engine : MonoBehaviour
{
    private Rigidbody parentRb;
    private Rigidbody selfRb;

    public int power = 0;
    
    public int maxPower = 
    private void Start()
    {
        parentRb = gameObject.GetComponentInParent<Rigidbody>();
        selfRb = gameObject.GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
                
    }
}
