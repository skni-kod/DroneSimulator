using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class AutoTunerConnector : MonoBehaviour
{
    public Rigidbody underSimulation;

    private TcpListener _listener;
    private Thread _thread;
    
    public string ip;
    public int port;

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
