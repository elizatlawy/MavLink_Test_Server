using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

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
                // Console.WriteLine("UDP Server Got: " + message.ToString());
                // Console.WriteLine("UDP Server message data: " + message.data.ToString());
                // Console.WriteLine("UDP Server message msgtypename: " + message.msgtypename.ToString());
                switch (message.data)
                {
                    case MAVLink.mavlink_param_request_read_t pm:
                    {
                        Console.WriteLine("UDP Server message vlink_param_request_read_t: " + ASCIIEncoding.ASCII.GetString(pm.param_id));
                        break;
                    }
                }
                Console.WriteLine("UDP Server GOT message msgtypename: " + message.data);

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
                        //custom_mode = 2730
                        custom_mode = 16383
                    }, false, 1, 155, 0);
                Send(packet);
                Console.WriteLine("UDPServer sent HB");
            }
        }

        public void SendAll()
        {
            for (int i = 1; i < 17; i++)
            {
                SendAUTOPILOT_VERSION(i);
                // Console.WriteLine("Press any key to SEND MOT_CHNL_IN ParamMessage number: " + i);
                // System.Console.ReadKey();
                SendParamMessage(i,"MOT_CHNL_IN");
                // Console.WriteLine("Press any key to SEND SERVO MIN + MAX ParamMessage number: " + i);
                // System.Console.ReadKey();
                string servoMin = "SERVO" + i + "_MIN";
                string servoMax = "SERVO" + i + "_MAX";
                int minStart = 1000 + i;
                int maxStart = 1900 + i;
                SendParamMessage(minStart,servoMin);
                SendParamMessage(maxStart,servoMax);
                Thread.Sleep(500);
                // Console.WriteLine("Press any key to SEND ServoOutputRaw message number: " + i);
                // System.Console.ReadKey();
                SendServoOutputRaw();
            }
            
            // ushort[] voltage = {100, 101, 102, 103};
            // ushort[] current = {200, 201, 202, 203};
            // ushort[] totalcurrent = {0,0,0,0};
            // ushort[] rpm = {1000, 1001, 1002, 1003};
            // ushort[] count = {0, 0, 0, 0};
            // byte[] temperature = {25, 45, 65, 99};
            // ushort[] voltage5 = {555, 666, 102, 103};
            // SendESCMavLinkMsg1to4(voltage,current,totalcurrent,rpm,count,temperature);
            // SendESCMavLinkMsg5to8(voltage5,current,totalcurrent,rpm,count,temperature);
            // for (float i = 1; i <= 6; i++)
            // {
            //     SendMavLinkMsg(5000, i, 11.0F + i / 10, 22.0F + i / 10, 33.0F + i / 10, 44.0F + i / 10, 2340, 1170);
            // }
            // SendMavLinkMsg(5001, 55.77F, 66.77F, 77.77F, 0, 0, 2340, 1170);
            // SendMavLinkMsg(5002, 88.8F, 99.8F, 111.8F, 222.8F, 0, 2340, 1170);
            // SendMavLinkMsg(5003, 12.34F, 23.45F, 34.56F, 45.67F, 56.78F, 2340, 1170);
            // SendMavLinkMsg(5004, 0F, 1F, 0F, 1F, 0F, 2340, 1170);
            // SendMavLinkMsg(5005, 1.1F, 2.2F, 3.3F, 4.4F, 5.5F, 6.6F, 1170);
            // SendMavLinkMsg(5006, 6.6F, 21.12F, 31.13F, 4.4F, 5.5F, 2340, 1170);
            // SendMavLinkMsg(5007, 98.7F, 87.6F, 76.5F, 65.4F, 54.3F, 43.2F, 1170);
            // SendMavLinkMsg(5008, 111.1F, 87.6F, 76.5F, 65.4F, 54.3F, 2340, 1170);
            // SendMavLinkMsg(5010, 2730, 333.33F, 1F, 46F, 55F, 2340, 1170);
            //  float initial = 5000;
            //  for (float i = 200; i  >= 0; i = i - 10)
            //  {
            //      Thread.Sleep(50);
            //      SendMavLinkMsg(5011, 100F, 0, 1F, i, initial, 0, 0);
            //      Console.WriteLine("UDPServer sent 5011 message, curr Fuel is: " + i);
            //      SendMavLinkMsg(5011, 100F, 333.33F, 1F, i, initial, 2340, 1170);
            //  }
             // float initial = 100;
             // float curr = 50;
             // SendMavLinkMsg(5011, 100F, 333.33F, 1F, curr, initial, 2340, 1170);
             // SendMavLinkMsg(5012, 19F, 28F, 37F, 46F, 55F, 2340, 1170);
             // SendMavLinkMsg(5020, 19F, 28F, 37F, 46F, 55F, 2340, 1170);

        }
        readonly object readlock = new object();
        public void SendMavLinkMsg(ushort pcommand, float p1, float p2, float p3, float p4,
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

        public void SendParamMessage(float paramValue, string paramName)
        {
            lock (readlock)
            {
                byte[] bytesArray = Encoding.UTF8.GetBytes(paramName);
                Array.Resize(ref bytesArray, 16);
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.PARAM_VALUE,
                    new MAVLink.mavlink_param_value_t()
                    {
                        param_id = bytesArray,
                        param_value = paramValue,

                    }, false, 1, 155, 0);
                Send(packet);
                Console.WriteLine("UDPServer sent mavlink_param_value_t with param: " + paramValue);
            }
        }
        public void SendServoOutputRaw()
        {
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW,
                    new MAVLink.mavlink_servo_output_raw_t()
                    {
                        servo1_raw = 1100,
                        servo2_raw = 1200,
                        servo3_raw = 1300,
                        servo4_raw = 1400,
                        servo5_raw = 1500,
                        servo6_raw = 1600,
                        servo7_raw = 1700,
                        servo8_raw = 1800,
                        servo9_raw = 1900,

                    }, false, 1, 155, 0);
                Send(packet);
                Console.WriteLine("UDPServer sent mavlink_param_value_t");
            }
        }


        public void SendAUTOPILOT_VERSION(int i)
        {   
            lock (readlock)
            {
                string n = i.ToString();
                byte[] bytesArray = Encoding.UTF8.GetBytes(("0." + n));
                Array.Resize(ref bytesArray, 8);
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.AUTOPILOT_VERSION,
                    new MAVLink.mavlink_autopilot_version_t()
                    {
                        capabilities = 1,
                        uid = 1,
                        flight_sw_version = 1,
                        middleware_sw_version = 1,
                        os_sw_version = 1,
                        board_version = 1,
                        vendor_id = 1,
                        product_id = 1,
                        flight_custom_version = bytesArray,
                        // middleware_custom_version = new byte[] { 1, 0, 0, 0, 0, 0, 0, 0 },
                        // os_custom_version = new byte[] { 1, 0, 0, 0, 0, 0, 0, 0 },
                        // uid2 = new byte[] { 1, 0, 0, 0, 0, 0, 0, 0 },
                        
                        
                    }, false, 1, 155, 0);
                Send(packet);
                Console.WriteLine("UDPServer sent mavlink_autopilot_version_t");
            }
        }
        public void SendESCMavLinkMsg1to4(ushort[] _voltage,ushort[] _current,ushort[] _totalcurrent,ushort[] _rpm,ushort[] _count,byte[] _temperature)
        {   
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.ESC_TELEMETRY_1_TO_4,
                    new MAVLink.mavlink_esc_telemetry_1_to_4_t()
                    {
                        voltage = _voltage,
                        current = _current,
                        totalcurrent = _totalcurrent,
                        rpm = _rpm,
                        count = _count,
                        temperature = _temperature,
                    }, false, 1, 155, 0);
                Send(packet);
                Console.WriteLine("UDPServer sent SCMavLinkMsg1to4");
            }
        }
        public void SendESCMavLinkMsg5to8(ushort[] _voltage,ushort[] _current,ushort[] _totalcurrent,ushort[] _rpm,ushort[] _count,byte[] _temperature)
        {   
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.ESC_TELEMETRY_5_TO_8,
                    new MAVLink.mavlink_esc_telemetry_5_to_8_t()
                    {
                        voltage = _voltage,
                        current = _current,
                        totalcurrent = _totalcurrent,
                        rpm = _rpm,
                        count = _count,
                        temperature = _temperature,
                    }, false, 1, 155, 0);
                Send(packet);
                Console.WriteLine("UDPServer sent SCMavLinkMsg5to8");
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
                    //    Console.WriteLine("UDP Server Error", exp.Message + "\n" +
                    //         "server must recieve bytes from client (to get client address) before trying to send bytes");
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