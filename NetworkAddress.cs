using System;
using System.Net;
using System.Net.Sockets;

namespace MavlinkTestServer
{
    class NetworkAddress
    {
        public NetworkAddress() { }

        public NetworkAddress(string ip, string port)
        {
            IP = StringToIPAddress(ip);
            Port = PortStringToInt(port);
        }

        public NetworkAddress(string ip, int port)
        {
            IP = StringToIPAddress(ip);
            Port = port;
        }

        public NetworkAddress(IPAddress ip, int port)
        {
            IP = ip;
            Port = port;
        }

        private IPAddress ip_;
        public IPAddress IP
        {
            get { return ip_; }
            set { ip_ = value; }
        }

        private int port_;
        public int Port
        {
            get { return port_; }
            set { port_ = value; }
        }

        public IPEndPoint EndPoint()
        {
            return new IPEndPoint(IP, Port);
        }

        public IPAddress StringToIPAddress(string ip)
        {
            IPAddress temp_ip = null;
            if (!IPAddress.TryParse(ip, out temp_ip))
            {
                return null; //error parsing
            }
            return temp_ip;
        }

        public static string GetLocalIPAddress()
        {
            using (Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, 0))
            {
                try
                {
                    socket.Connect("10.0.0.255", 65530);
                    IPEndPoint endPoint = socket.LocalEndPoint as IPEndPoint;
                    return endPoint.Address.ToString();
                }
                catch (SocketException)
                {
                    return null;
                }
            }
        }

        public static int PortStringToInt(string port)
        {
            int parsed_port;
            try
            {
                parsed_port = int.Parse(port);
                if (!IsPortValid(parsed_port))
                    parsed_port = -1; //error parsing
            }
            catch (FormatException)
            {
                parsed_port = -1; //error parsing
            }
            return parsed_port;
        }

        public static bool IsPortValid(int port)
        {
            if (port <= 65535)
                return true;
            return false;
        }

        public bool IsIpValid() { return IP != null; }
        public bool IsPortValid() { return IsPortValid(Port); }

    }
}
