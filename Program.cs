using System.Threading;

namespace MavlinkTestServer
{
    class Program
    {
        static void Main()
        {
            UDPServerCom udpServer = new UDPServerCom(5000);
            //TCPServer tcpServer = new TCPServer();

            //SerialCom sr = new SerialCom();

            //sr.Connect("COM6", 57600);
            int i = 0;
            while (true)
            {
                //udpServer.SendHB();
                //sr.SendHB();
                //tcpServer.SendHB();
                Thread.Sleep(500);
                ++i;
                if (i >= 1)
                {
                    i = 0;
                    //sr.SendAll();
                    //tcpServer.SendAll();
                    udpServer.SendAll();

                }
            }
        }

    }
}
   