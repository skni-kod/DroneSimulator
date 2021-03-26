using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using UnityEngine.InputSystem.HID;

public class AutoTunerConnector : MonoBehaviour
{
    public Rigidbody underSimulation;

    private TcpListener _listener;
    private Thread _thread;
    
    public string ip;
    public int port;

    PIDAutotuner tuner = new PIDAutotuner();
    private long microseconds;
    private bool printed = false;
    public bool disabled = false;
    
    public void Start()
    {
        tuner.setTargetInputValue(0f);
        tuner.setLoopInterval((long)(Time.fixedDeltaTime * 1E6));
        tuner.setOutputRange(-20, 20);
        tuner.setZNMode(ZNMode.ZNModeBasicPID);
        tuner.startTuningLoop(DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000);
        tuner.setTuningCycles(100);
    }

    public void FixedUpdate()
    {
        if (disabled) return;
        if(!tuner.isFinished())
            Tune();
        else if (!printed)
        {
            Debug.Log("Kp = " + tuner.getKp());
            Debug.Log("Ki = " + tuner.getKi());
            Debug.Log("Kd = " + tuner.getKd());
            printed = true;
        }
    }

    [ContextMenu("Tune")]
    public void Tune()
    {
        long prevMicroseconds = microseconds;
        microseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000;
        float input = underSimulation.angularVelocity.y;
        double output = tuner.tunePID(input, microseconds);
        underSimulation.GetComponent<TestTorque>().yaw = (float)output;
    }

    [ContextMenu("Bind")]
    public void Connect()
    {
        _listener = new TcpListener(Dns.GetHostAddresses(ip)[0], port);
        _listener.Start();
        _thread = new Thread(ListenerThread);
        _thread.Start();
    }

    [ContextMenu("Disconnect")]
    public void Disconnect()
    {
        
    }

    public void ListenerThread()
    {
        Byte[] bytes = new Byte[256];
        String data = null;
        while (true)
        {
            TcpClient client = _listener.AcceptTcpClient();
            NetworkStream stream = client.GetStream();
            while (client.Connected)
            {
                stream.Read(bytes, 0, 4 * sizeof(float));
                var u = BitConverter.ToSingle(bytes, 0);
                var p = BitConverter.ToSingle(bytes, 4);
                var i = BitConverter.ToSingle(bytes, 8);
                var d = BitConverter.ToSingle(bytes, 12);
            }
            
            client.Close();
        }
    }
}
