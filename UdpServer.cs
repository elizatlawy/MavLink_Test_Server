using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;

namespace MavlinkTestServer
{
    public class UDPServerCom : UdpClient
    {
        private IPEndPoint endpoint_;
        List<Action<byte[]>> subsFuncs_ = new List<Action<byte[]>>();
        bool isDead_ = false;
        readonly MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();

        public bool IsDead { get { return isDead_; } }

        public UDPServerCom(int port) : base(new IPEndPoint(IPAddress.Parse("127.0.0.1"), port)) 
        {
            endpoint_ = new IPEndPoint(IPAddress.Any, port);
            AsyncListen();
            Console.WriteLine(" UDP Server start Listening...");
        }

        private void AsyncListen()
        {
            if (!IsDead)
                BeginReceive(new AsyncCallback(OnIncomingBytes), null);
        }
    

        private void OnIncomingBytes(IAsyncResult res)
        {
            try
            {
                byte[] bytes = EndReceive(res, ref endpoint_);
                MAVLink.MAVLinkMessage message = new MAVLink.MAVLinkMessage(bytes);
                Console.WriteLine(message.ToString());
                //Publish(bytes);
                AsyncListen(); //keep listening
            }
            catch (System.ObjectDisposedException exp)
            {
                isDead_ = true;
            }
            catch (SocketException exp)
            {
                //other side disconnected
            }
        }

        public void CloseMe()
        {
            isDead_ = true;
            Close();
        }

        public void SendHB()
        {
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT,
                    new MAVLink.mavlink_heartbeat_t()
                    {

                        autopilot = (byte) MAVLink.MAV_AUTOPILOT.INVALID,
                        base_mode = 0,
                        system_status = 0,
                        type = (byte) MAVLink.MAV_TYPE.GCS,
                        custom_mode = 2730
                    }, false, 1, 155, 0);
                Send(packet);
            }
        }

        public void SendAll()
        {
            for (float i = 1; i <= 6; i++)
            {
                SendMaveLinkMsg(5000, i, 11.0F + i / 10, 22.0F + i / 10, 33.0F + i / 10, 44.0F + i / 10, 2340, 1170);
            }
            SendMaveLinkMsg(5001, 55.77F, 66.77F, 77.77F, 0, 0, 2340, 1170);
            SendMaveLinkMsg(5002, 88.8F, 99.8F, 111.8F, 222.8F, 0, 2340, 1170);
            SendMaveLinkMsg(5003, 12.34F, 23.45F, 34.56F, 45.67F, 56.78F, 2340, 1170);
            SendMaveLinkMsg(5004, 0F, 1F, 0F, 1F, 0F, 2340, 1170);
            SendMaveLinkMsg(5005, 1.1F, 2.2F, 3.3F, 4.4F, 5.5F, 6.6F, 1170);
            SendMaveLinkMsg(5006, 6.6F, 21.12F, 31.13F, 4.4F, 5.5F, 2340, 1170);
            SendMaveLinkMsg(5007, 98.7F, 87.6F, 76.5F, 65.4F, 54.3F, 43.2F, 1170);
            SendMaveLinkMsg(5008, 111.1F, 87.6F, 76.5F, 65.4F, 54.3F, 2340, 1170);
            //Send(5009, 1F, 0F, 1F, 0F, 1F, 0F, 1170);
            SendMaveLinkMsg(5010, 2730, 333.33F, 1F, 46F, 55F, 2340, 1170);
            SendMaveLinkMsg(5011, 100F, 333.33F, 1F, 46F, 55F, 2340, 1170);
            SendMaveLinkMsg(5012, 19F, 28F, 37F, 46F, 55F, 2340, 1170);
        }
        readonly object readlock = new object();
        public void SendMaveLinkMsg(ushort pcommand, float p1, float p2, float p3, float p4,
            float p5, float p6, float p7)
        {   
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
                    new MAVLink.mavlink_command_long_t()
                    {
                        confirmation = 1,
                        command = pcommand,
                        param1 = p1,
                        param2 = p2,
                        param3 = p3,
                        param4 = p4,
                        param5 = p5,
                        param6 = p6,
                        param7 = p7,

                    }, false, 1, 155, 0);
                Send(packet);
                
            }
        }

        public bool Send(byte[] bytes)
        {
            if (!IsDead)
            {
                try
                {
                    int a = Send(bytes, bytes.Length, endpoint_);
                    return true;
                }
                catch (SocketException exp)
                {
                   Console.WriteLine("UDP Server Error", exp.Message + "\n" +
                        "server must recieve bytes from client (to get client address) before trying to send bytes");
                }
            }
            return false;
        }


        public void Subscribe(Action<byte[]> func)
        {
            subsFuncs_.Add(func);
        }

        public void Unsubscribe(Action<byte[]> func)
        {
            subsFuncs_.Remove(func);
        }

        public void Publish(byte[] bytes)
        {
            foreach (var func in subsFuncs_)
                func(bytes);
        }

    }
}