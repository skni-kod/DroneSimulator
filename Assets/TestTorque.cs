using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections.LowLevel.Unsafe;
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

    public PID pitchAnglePID = new PID();
    public PID rollAnglePID = new PID();

    public PID forwardVelocityPID = new PID();
    public PID rightVelocityPID = new PID();
    
    [UPyPlot.UPyPlotController.UPyProbe]
    public float output = 0.0f;

    [UPyPlot.UPyPlotController.UPyProbe] 
    public float target = 0.0f;
    
    public TypeOfInput typeOfInput = TypeOfInput.GAMEPAD;
    public Vector4 externalInput = new Vector4();

    public float yaw;
    public bool ownYawPid = true;
    
    public void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    public void FixedUpdate()
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
        var angularVelocity = transform.InverseTransformDirection(rb.angularVelocity);
        float currentRollVelocity = -angularVelocity.z;
        float currentYawVelocity = angularVelocity.y;
        float currentPitchVelocity = -angularVelocity.x;
        float currentPitchAngle = NormalizeAngle(-rb.rotation.eulerAngles.x, -180, 180);
        float currentRollAngle = NormalizeAngle(-rb.rotation.eulerAngles.z, -180, 180);
        var velocity = transform.InverseTransformDirection(rb.velocity);
        float currentforwardVelocity = -velocity.z;
        float currentrightVelocity = velocity.x;
        
        output = currentYawVelocity;

        float targetVerticalVelocity = input.x * 5f;
        float targetYawVelocity = input.y * Mathf.Deg2Rad * 200;
        float targetForwardVelocity = -input.w * 5;
        float targetRightVelocity = input.z * 5;
        
        float targetPitchAngle = forwardVelocityPID.CalculateCurrentAnswer(currentforwardVelocity, targetForwardVelocity);
        float targetRollAngle = rightVelocityPID.CalculateCurrentAnswer(currentrightVelocity, targetRightVelocity);
        
        float targetRollVelocity = rollAnglePID.CalculateCurrentAnswer(currentRollAngle, targetRollAngle);
        float targetPitchVelocity = pitchAnglePID.CalculateCurrentAnswer(currentPitchAngle, targetPitchAngle);
        
        target = targetYawVelocity;

        float thrust = thurstPID.CalculateCurrentAnswer(currentVerticalVelocity, targetVerticalVelocity);
        if(ownYawPid)
            yaw = yawPID.CalculateCurrentAnswer(currentYawVelocity, targetYawVelocity);
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
        
        Debug.Log("Inertia Tensor: " + rb.inertiaTensor);
        Debug.Log("Inertia Tensor Rotation: " + rb.inertiaTensorRotation.eulerAngles);
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
        
        Debug.Log("Mfr: " + mfr + " Mfl: " + mfl + " Mbl: " + mbl + " Mbr: " + mbr );
        
        rb.AddForceAtPosition(this.transform.up * mfr, mfrTransform.position);
        rb.AddForceAtPosition(this.transform.up * mfl, mflTransform.position);
        rb.AddForceAtPosition(this.transform.up * mbr, mrrTransform.position);
        rb.AddForceAtPosition(this.transform.up * mbl, mrlTransform.position);
        
        Debug.DrawLine(mfrTransform.position, mfrTransform.position + this.transform.up * mfr);
        Debug.DrawLine(mflTransform.position, mflTransform.position + this.transform.up * mfl);
        Debug.DrawLine(mrrTransform.position, mrrTransform.position + this.transform.up * mbr);
        Debug.DrawLine(mrlTransform.position, mrlTransform.position + this.transform.up * mbl);
        Debug.DrawLine(transform.position, transform.position + transform.forward * 3, Color.red);

        //float torque = mfr + mbl - mfl - mbr;
        float torque = -mfr - mbl + mfl + mbr;
        rb.AddRelativeTorque(transform.up * 1f * torque);
    }
    
    public float NormalizeAngle(float value, float start, float end)
    {
        float width = end - start;
        float offsetValue = value - start;
        return ( offsetValue - ( Mathf.Floor( offsetValue / width ) * width ) ) + start ;
    }
}