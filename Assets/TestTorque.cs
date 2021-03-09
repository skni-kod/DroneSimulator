using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

public class TestTorque : MonoBehaviour
{
    public enum TypeOfInput
    {
        GAMEPAD, EXTERNAL
    }
    
    [System.Serializable]
    public class PID
    {
        private float[] errors = {0.0f, 0.0f, 0.0f};
        private float[] answers = {0.0f, 0.0f};
        private float intergralTerm = 0.0f;

        public float Ki = 1.0f;
        public float Kp = 1.0f;
        public float Kd = 1.0f;
        public float OutputMax = 20.0f;
        public float OutputMin = 0.0f;
        
        public float CalculateCurrentAnswer(float current, float target)
        {
            float error = target - current;
            errors[2] = errors[1];
            errors[1] = errors[0];
            errors[0] = error;
            
            //Debug.Log("===============");
            //Debug.Log("Error: " + error);

            intergralTerm += Ki * error * Time.fixedDeltaTime;
            intergralTerm = Mathf.Clamp(intergralTerm, OutputMin, OutputMax);

            float dInput = errors[0] - errors[1];
            float deriativeTerm = Kd * (dInput / Time.fixedDeltaTime);

            float proportionalTerm = Kp * error;

            float output = proportionalTerm + intergralTerm - deriativeTerm;

            output = Mathf.Clamp(output, OutputMin, OutputMax);
            
            return output;
        }
    }


    public Transform mfrTransform;
    public Transform mflTransform;
    public Transform mrlTransform;
    public Transform mrrTransform;
    
    
    public Rigidbody rb;
    public PID thurstPID = new PID();
    public PID rollPID = new PID();
    public PID yawPID = new PID();
    public PID pitchPID = new PID();
    
    [UPyPlot.UPyPlotController.UPyProbe]
    public float output = 0.0f;

    [UPyPlot.UPyPlotController.UPyProbe] 
    public float target = 0.0f;
    
    public TypeOfInput typeOfInput = TypeOfInput.GAMEPAD;
    public Vector4 externalInput = new Vector4();

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        Vector4 input = new Vector4();

        if (typeOfInput == TypeOfInput.GAMEPAD)
        {
            var gamepad = Gamepad.current;
            if (gamepad != null)
            {
                input.x = gamepad.rightStick.y.ReadValue();
                input.y = gamepad.rightStick.x.ReadValue();
                input.z = gamepad.leftStick.x.ReadValue();
                input.w = gamepad.leftStick.y.ReadValue();
            }
        }
        else
        {
            input = externalInput;
        }
        
        float currentVerticalVelocity = rb.velocity.y;
        var velocity = transform.InverseTransformDirection(rb.angularVelocity);
        float currentRollVelocity = -velocity.z;
        float currentYawVelocity = -velocity.y;
        float currentPitchVelocity = -velocity.x;
        
        output = currentVerticalVelocity;

        float targetVerticalVelocity = input.x * 5f;
        float targetYawVelocity = -input.y;
        float targetRollVelocity = input.z;
        float targetPitchVelocity = -input.w;
        
        target = targetVerticalVelocity;

        float thrust = thurstPID.CalculateCurrentAnswer(currentVerticalVelocity, targetVerticalVelocity);
        float yaw = yawPID.CalculateCurrentAnswer(currentYawVelocity, targetYawVelocity);
        float pitch = pitchPID.CalculateCurrentAnswer(currentPitchVelocity, targetPitchVelocity);
        float roll = rollPID.CalculateCurrentAnswer(currentRollVelocity, targetRollVelocity);

        //Debug.Log("++++++++++++++++++++++++++++++++++++++++++++++");
        //Debug.Log(currentRollVelocity);
        //Debug.Log(currentYawVelocity);
        //Debug.Log(currentPitchVelocity);
        
        //Debug.Log("Thurst: " + thrust);
        //Debug.Log("VerticalVelocity: " + currentVerticalVelocity);
        //Debug.Log("++++++++++++++++++++++++++++++++++++++++++++++");
        
        MotorMixing(thrust, yaw, pitch, roll);
    }

    
    public void MotorMixing(float thrust, float yaw, float pitch, float roll)
    {
        /*var mfr = thrust + yaw + pitch + roll;
        var mfl = thrust - yaw + pitch - roll;
        var mbr = thrust - yaw - pitch + roll;
        var mbl = thrust + yaw - pitch - roll;*/

        var mfr = thrust + pitch - roll - yaw;
        var mfl = thrust + pitch + roll + yaw;
        var mbl = thrust - pitch + roll - yaw;
        var mbr = thrust - pitch - roll + yaw;

        mfr = Mathf.Clamp(mfr, 0, 20);
        mfl = Mathf.Clamp(mfl, 0, 20);
        mbr = Mathf.Clamp(mbr, 0, 20);
        mbl = Mathf.Clamp(mbl, 0, 20);
        
        rb.AddForceAtPosition(this.transform.up * mfr, mfrTransform.position);
        rb.AddForceAtPosition(this.transform.up * mfl, mflTransform.position);
        rb.AddForceAtPosition(this.transform.up * mbr, mrrTransform.position);
        rb.AddForceAtPosition(this.transform.up * mbl, mrlTransform.position);
        
        Debug.DrawLine(mfrTransform.position, mfrTransform.position + this.transform.up * mfr);
        Debug.DrawLine(mflTransform.position, mflTransform.position + this.transform.up * mfl);
        Debug.DrawLine(mrrTransform.position, mrrTransform.position + this.transform.up * mbr);
        Debug.DrawLine(mrlTransform.position, mrlTransform.position + this.transform.up * mbl);

        float torque = mfr + mbl - mfl - mbr;
        rb.AddRelativeTorque(transform.up * 0.8f * torque);
    }
}