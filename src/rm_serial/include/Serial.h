#ifndef _SERIAL_H
#define _SERIAL_H

#include<iostream>
#include<vector>
#include<queue>
#include<boost/asio.hpp>
#include<boost/bind.hpp>
#include <chrono>
using namespace std;
using namespace boost::asio;

#define FRAME_HEAD 0xFC
#define FRAME_END 0xFD
#define MSG_IMU 0x40
#define IMU_LEN 0x38
#define AHRS_LEN 0x30
#define MSG_AHRS 0x41
#define PI 3.1415926535

class SerialPort
{
public:
	io_service m_IoService;
	boost::system::error_code ec;
	serial_port m_serialPort;
	int flag, open_flag, buffer_flag;
	double quat[4],gyro[3],accel[3];
	vector<unsigned char> data_read;
	vector<unsigned char> buffer_data;
	int count;//��ˮ��
	float  Heading, Pitch, Roll;//�����ǻ���
	float heading, pitch, roll;//������
	float Qw, Qx, Qy, Qz;//��Ԫ��
	long long int Timestamp;//ʱ���
	unsigned char data_one;
	vector<vector<int>>Angle;
	vector<int> Rm_Serial_Data;

	SerialPort(const char* dev_name, int paket_len);
	bool isSucceed(int _flag);
	bool Sendcom(int pitch, int yaw,int signal);
	vector<vector<int>>Get_Pitch_Yaw(vector<unsigned char>& _data);
	vector<vector<int>>Get_P_Y_R(vector<unsigned char>& _data);
	vector<int>Get_Imu_C_Dj(vector<unsigned char>& _data);
	bool Readcom();
	bool Readcom_imu(int data_len);
	void show_data();
	void  readn100(unsigned char* data, bool* isuseful);
	long long timestamp(unsigned char Data_1, unsigned char Data_2, unsigned char Data_3, unsigned char Data_4);//��ȡʱ���
	long long timestamp(unsigned char data_1, unsigned char data_2, unsigned char data_3, unsigned char data_4, unsigned char data_5, unsigned char data_6, unsigned char data_7, unsigned char data_8);
	float data_trans(unsigned char data_1, unsigned char data_2, unsigned char data_3, unsigned char data_4);
	void  readcomn100();
	void ProtolOfSerial(int euler_angle[3], int angular_velocity[3], int accelerated_velocity[3], char camp, char shoot_flag);
	void send_com_test();
};

#define NO 9999999

SerialPort::SerialPort(const char* dev_name, int paket_len) :m_serialPort(m_IoService), data_read(paket_len)
{
	cout << "Init by:" << paket_len << endl;
	buffer_flag = 0;
	flag = 0;
	flag |= m_serialPort.open(dev_name, ec).failed();
	flag |= m_serialPort.set_option(serial_port::baud_rate(115200), ec).failed();
	flag |= m_serialPort.set_option(serial_port::flow_control(serial_port::flow_control::none), ec).failed();
	flag |= m_serialPort.set_option(serial_port::parity(serial_port::parity::none), ec).failed();
	flag |= m_serialPort.set_option(serial_port::stop_bits(serial_port::stop_bits::one), ec).failed();
	flag |= m_serialPort.set_option(serial_port::character_size(8), ec).failed();

	if (flag)
	{
		cout << "Serial port opening failure" << endl;
		open_flag = 1;
        exit(-1);
	}
	else
	{
		cout << "Serial port opening successfully!" << endl;
	}
}

bool SerialPort::isSucceed(int _flag)
{
	if (_flag == 0)return 0;
	else return 1;
}

bool SerialPort::Sendcom(int pitch, int yaw,int signal)
{
	unsigned char data_write[8] = { 0xFF ,0x00 ,0x00 ,0x00 ,0x00 ,0x01 ,0x0 ,0xFE,
								  };
	int high_pitch = 0;
	int low_pitch = 0;
	int high_yaw = 0;
	int low_yaw = 0;

	if (pitch >= 0)
	{
		high_pitch = pitch >> 8;
		low_pitch = pitch & 0xFF;
	}
	else
	{
		pitch = -pitch;
		high_pitch = (pitch >> 8) | 0x80;
		low_pitch = pitch & 0xFF;
	}

	if (yaw >= 0)
	{
		high_yaw = yaw >> 8;
		low_yaw = yaw & 0xFF;
	}
	else
	{
		yaw = -yaw;
		high_yaw = (yaw >> 8) | 0x80;
		low_yaw = yaw & 0xFF;
	}

	data_write[1] = high_pitch;
	data_write[2] = low_pitch;
	data_write[3] = high_yaw;
	data_write[4] = low_yaw;
	data_write[6] = signal;
	//if (roll != NO)
	//{
	//	if (roll >= 0)
	//	{
	//		high_roll = roll >> 8;
	//		low_roll = roll & 0xFF;
	//	}
	//	else
	//	{
	//		roll = -roll;
	//		high_roll = roll >> 8;
	//		low_roll = roll & 0xFF;
	//		high_roll += 128;
	//	}

	//	data_write[5] = high_roll;
	//	data_write[6] = low_roll;
	//}

	size_t len_write = this->m_serialPort.write_some(buffer(data_write), ec);
	//cout << "二进制数据包:" << int(data_write[1]) << " " << int(data_write[2]) << " " << int(data_write[3]) << " "<<int(data_write[4]) << endl;
	//size_t len = write(m_serialPort, buffer(data_write), ec);

	if (!ec)
	{
		//cout << "Send is ok!" << endl;
		return true;
	}
	else
	{
		cout << "Send is fail!" << endl;
		return false;
	}

} 

vector<vector<int>>SerialPort::Get_Pitch_Yaw(vector<unsigned char>& _data)
{
	vector<vector<int>>angle_temp;
	vector<int>temp;
	int pitch, yaw;

	if (_data[0] == 0xFF && _data[5] == 0x01 && _data[6] == 0x00 && _data[7] == 0xFE && _data.size() == 8)
	{
		//���p
		if ((_data[1] & 0x80) == 0x80)
		{
			pitch = -((_data[1] & 0x7F) * 256 + _data[2]);
			temp.push_back(pitch);
		}
		else
		{
			pitch = (_data[1] & 0x7F) * 256 + _data[2];
			temp.push_back(pitch);
		}
		//���y
		if ((_data[3] & 0x80) == 0x80)
		{
			yaw = -((_data[3] & 0x7F) * 256 + _data[4]);
			temp.push_back(yaw);
		}
		else
		{
			yaw = (_data[3] & 0x7F) * 256 + _data[4];
			temp.push_back(yaw);
		}

		angle_temp.push_back(temp);
	}
	return angle_temp;
}

vector<vector<int>>SerialPort::Get_P_Y_R(vector<unsigned char>& _data)
{
	vector<vector<int>>angle_temp;
	vector<int>temp;
	int pitch, yaw, roll;

	if (_data[0] == 0xFF && _data[7] == 0xFE && _data.size() == 8)
	{
		//���p
		if ((_data[1] & 0x80) == 0x80)
		{
			pitch = -((_data[1] & 0x7F) * 256 + _data[2]);
			temp.push_back(pitch);
		}
		else
		{
			pitch = (_data[1] & 0x7F) * 256 + _data[2];
			temp.push_back(pitch);
		}
		//���y
		if ((_data[3] & 0x80) == 0x80)
		{
			yaw = -((_data[3] & 0x7F) * 256 + _data[4]);
			temp.push_back(yaw);
		}
		else
		{
			yaw = (_data[3] & 0x7F) * 256 + _data[4];
			temp.push_back(yaw);
		}
		//���r
		if ((_data[5] & 0x80) == 0x80)
		{
			roll = -((_data[5] & 0x7F) * 256 + _data[6]);
			temp.push_back(roll);
		}
		else
		{
			roll = (_data[5] & 0x7F) * 256 + _data[6];
			temp.push_back(roll);
		}

		angle_temp.push_back(temp);
	}

	return angle_temp;

}

vector<int>SerialPort::Get_Imu_C_Dj(vector<unsigned char>& _data)
{
	vector<int> rmserial;
	double num = 0;
	//分别从 _data 的不同位置解析出 3 个 24 位有符号整数，并将它们存入 rmserial 中
	for(int i = 0; i < 3; i++)
	{
		if((_data[3*i+1] & 0x80) == 0x80)
			num = -((_data[3*i+1] & 0x7f)*256*256 + _data[3*i+2] * 256 + _data[3*i+3] * 1);
		else
			num = _data[3*i+1]*256*256 + _data[3*i+2] * 256 + _data[3*i+3] * 1;
		rmserial.push_back(num);	
	}
	for(int i = 0; i < 3; i++)
	{
		if((_data[3*i+11] & 0x80) == 0x80)
			num = -((_data[3*i+11] & 0x7f)*256*256 + _data[3*i+12] * 256 + _data[3*i+13] * 1);
		else
			num = _data[3*i+11]*256*256 + _data[3*i+12] * 256 + _data[3*i+13] * 1;
		rmserial.push_back(num);	
	}
	for(int i = 0; i < 3; i++)
	{
		if((_data[3*i+21] & 0x80) == 0x80)
			num = -((_data[3*i+21] & 0x7f)*256*256 + _data[3*i+22] * 256 + _data[3*i+23] * 1);
		else
			num = _data[3*i+21]*256*256 + _data[3*i+22] * 256 + _data[3*i+23] * 1;
		rmserial.push_back(num);	
	}

	return rmserial;
}


bool SerialPort::Readcom()
{
	//data_read.clear();

	size_t len_read = this->m_serialPort.read_some(buffer(data_read), ec);
	for(int i = 0; i < 8; i++)
		cout << int(data_read[i]) << " ";
	cout << endl;
	if (!ec)
	{
		Angle = Get_Pitch_Yaw(data_read);
		return true;
	}
	else
	{
		cout << "Read is fail!" << endl;
		return false;
	}
}

unsigned char data[32] = {0};
// 欧拉角r p y 角速度 x y z 加速度 x y z 阵营(r or b) 击打目标(a of f) 
void SerialPort::ProtolOfSerial(int euler_angle[3], int angular_velocity[3], int accelerated_velocity[3], char camp, char shoot_flag)
{
	data[0] = 0xff;
	data[28] = 0x88;
	data[31] = 0xfe;

	//r代表红方 b代表蓝方
	if (camp == 'r')
		data[29] = 0x01;
	else if (camp == 'b')
		data[29] = 0x02;
	//a代表打装甲板 f代表打符
	if (shoot_flag == 'a')
		data[30] = 0x01;
	else if (shoot_flag == 'f')
		data[30] = 0x02;

	for (int i = 0; i < 3; i++)
	{
		char euler_H, angular_H, accelerated_H;
		char euler_M, angular_M, accelerated_M;
		char euler_L, angular_L, accelerated_L;
		
		//欧拉角
		if (euler_angle[i] < 0)
		{
			int _ = -euler_angle[i];
			euler_L = _ & 0xff;
			euler_M = (_ >> 8) & 0xff;
			euler_H = (_ >> 16) | 0x80;
		}
		else
		{
			euler_L = euler_angle[i] & 0xff;
			euler_M = (euler_angle[i] >> 8) & 0xff;
			euler_H = (euler_angle[i] >> 16) & 0xff;
		}
		//角速度
		if (angular_velocity[i] < 0)
		{
			int _ = -angular_velocity[i];
			angular_L = _ & 0xff;
			angular_M = (_ >> 8) & 0xff;
			angular_H = (_ >> 16) | 0x80;
		}
		else
		{
			angular_L = angular_velocity[i] & 0xff;
			angular_M = (angular_velocity[i] >> 8) & 0xff;
			angular_H = (angular_velocity[i] >> 16) & 0xff;
		}
		//加速度
		if (accelerated_velocity[i] < 0)
		{
			int _ = -accelerated_velocity[i];
			accelerated_L = _ & 0xff;
			accelerated_M = (_ >> 8) & 0xff;
			accelerated_H = (_ >> 16) | 0x80;
		}
		else
		{
			accelerated_L = accelerated_velocity[i] & 0xff;
			accelerated_M = (accelerated_velocity[i] >> 8) & 0xff;
			accelerated_H = (accelerated_velocity[i] >> 16) & 0xff;
		}

		data[i * 3 + 1] = euler_H;
		data[i * 3 + 2] = euler_M;
		data[i * 3 + 3] = euler_L;

		data[i * 3 + 10] = angular_H;
		data[i * 3 + 11] = angular_M;
		data[i * 3 + 12] = angular_L;

		data[i * 3 + 19] = accelerated_H;
		data[i * 3 + 20] = accelerated_M;
		data[i * 3 + 21] = accelerated_L;

	}
}

void SerialPort::send_com_test()
{
	int angle[3]={1570000,-1570000,0};
	int g[3]={0};
	int a[3]={0};
	ProtolOfSerial(angle,g,a,'b','f');
	size_t len_write = this->m_serialPort.write_some(buffer(data), ec);
	if (!ec)
	{
		//cout << "Send is ok!" << endl;
	}
	else
	{
		cout << "Send is fail!" << endl;
	}
}

queue<unsigned char> queue_data;
bool SerialPort::Readcom_imu(int data_len)
{
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	vector<unsigned char> data_temp(data_len);
	size_t len_read = this->m_serialPort.read_some(buffer(data_temp,data_len),ec);
	// cout << len_read << endl;
	// for(int i = 0; i < data_temp.size(); i++)
	// 	cout << int(data_temp[i]) << " " ;
	// cout << endl << data_temp.size()<<"-----------------------------------------" << data_temp.size() << endl;
	// data_temp.clear();
	if(ec) {
		ROS_ERROR("串口断开,主动结束节点以触发串口重启机制");
		ros::shutdown();	 // 优雅关闭ROS
		exit(EXIT_FAILURE);  // 强制自杀,打破死循环
		return false;
	}
	for(int i = 0;i < len_read;i++){
		buffer_data.push_back(data_temp[i]);
		// cout <<std::hex<< (int)data_temp[i] << '\t';
	}
		//buffer_data.insert(buffer_data.end(), data_read.begin(), data_read.end());
	// cout << "Buffer:" << buffer_data.size() << "       \r";
	bool frame_received = false;
	if(buffer_data.size() >= data_len)
	{	
		// cout << "Size1::" << buffer_data.size() << endl;
		// cout << "111" << endl;
		// cout << "buffer_data:" << endl;
		// for(int i = 0; i < buffer_data.size();i++)
		// 	cout << hex << int(buffer_data[i]) << "\t" ;
		// cout << endl << "Buffer-----------------------------------------" << endl;

//		cout << buffer_data.size() << endl;
//		cout << "****************" << endl;
		
		buffer_flag = 0;
		for(size_t i = 0; i + data_len <= buffer_data.size(); ++i){
			
			
			// cout << int(buffer_data[i]) << " " << int(buffer_data[i+10]) << " " << int(buffer_data[i+20]) << " " << int(buffer_data[i+30]) << " " << int(buffer_data[i+33])<< endl;
			if(buffer_data[i] == 0xff && buffer_data[i+10] == 0x66 && buffer_data[i+20] == 0x77 && buffer_data[i+30] == 0x88 && buffer_data[i+35] == 0xFE)
			{
				// cout << "第" << i << "次      \r" << endl;
				vector<unsigned char> enable_data;
				for(int j = 0; j < data_len; j++){
					enable_data.push_back(buffer_data[i+j]);
					//  cout << int(buffer_data[i+j]) << " ";
				}
				// cout <<endl<<"-----------------------------------------" << endl;


				Rm_Serial_Data = Get_Imu_C_Dj(enable_data);
				Rm_Serial_Data.push_back(int(buffer_data[i+31]));
				Rm_Serial_Data.push_back(int(buffer_data[i+32]));
				Rm_Serial_Data.push_back((int(buffer_data[i+33]) << 8) | int(buffer_data[i+34]));
				frame_received = true;
				buffer_data.clear();
				break;
			}	
		}
		if(buffer_data.size() >= 200)
			buffer_data.clear();
		//cout << "fl" << endl;
		return frame_received;
	}
	else
	{

		return false;
	}

}


void SerialPort::show_data()
{
	for (int i = 0; i < 8; i++)
	{
		cout << int(this->data_read[i]) << " ";
	}
	cout << endl;
}



/*-------------------------------------N100������---------------------------------------------*/

	void  SerialPort::readn100(unsigned char* data, bool* isuseful)//��Ϊ����ֻ��Ҫahrs������ݣ���������ֻ��ȡahrs
	{
		bool istype = false;
		bool isimu = false;
		bool isarhs = false;
		unsigned char receive_data[80];
		int count = 0;//�����������ĳ���
		while (1)//����֡ͷ��֡β���������õ�receive_data������
		{
			size_t len;
			len = this->m_serialPort.read_some(buffer(data_read), ec);
			
			if (ec)
			{
				cout << "read fail" << endl;
				return;
			}

			for (int i = 0; i < data_read.size(); i++)
			{
				receive_data[count] = data_read[i];
				if (data_read[i] == FRAME_HEAD || count > 0)
				{
					count++;
				}
				if (count == 3) istype = true;
			}
			if (istype)
			{
				istype = false;
				//�ж���������
				if (receive_data[1] == MSG_IMU && receive_data[2] == IMU_LEN) return;
				else if (receive_data[1] == MSG_AHRS && receive_data[2] == AHRS_LEN) isarhs = true;
				else return;
			}
			if (count >= 56) break;
		}
		if (isarhs && receive_data[55] == FRAME_END)
		{
			memcpy(data, receive_data, 56);
			*isuseful = true;
		}
		else *isuseful = false;
		return;
	}

	long long SerialPort::timestamp(unsigned char data_1, unsigned char data_2, unsigned char data_3, unsigned char data_4, unsigned char data_5, unsigned char data_6, unsigned char data_7, unsigned char data_8)//��ȡʱ���
	{
		long long temp; //32λ16������
		uint16_t hh_16, h_16, l_16, ll_16; //��Ÿ�16λ����16λ
		uint32_t h_32, l_32;

		hh_16 = data_1 << 8 | data_2;
		h_16 = data_3 << 8 | data_4;
		l_16 = data_5 << 8 | data_6;
		ll_16 = data_7 << 8 | data_8;

		h_32 = (uint16_t)hh_16 << 16 | (uint16_t)h_16;
		l_32 = (uint16_t)l_16 << 16 | (uint16_t)ll_16;
		//���ںϳ�16λ��������ϳ�32λ����
		temp = (uint32_t)h_32 << 32 | (uint32_t) l_32;

		return temp;
	}

	long long SerialPort::timestamp(unsigned char Data_1, unsigned char Data_2, unsigned char Data_3, unsigned char Data_4)//��ȡʱ���
	{
		unsigned long temp; //32λ16������
		uint16_t H_16, L_16; //��Ÿ�16λ����16λ

		H_16 = Data_1 << 8 | Data_2;
		L_16 = Data_3 << 8 | Data_4;

		//���ںϳ�16λ��������ϳ�32λ����
		temp = (unsigned long)H_16 << 16 | (unsigned long)L_16;

		return temp;
	}

	float SerialPort::data_trans(unsigned char data_1, unsigned char data_2, unsigned char data_3, unsigned char data_4)
	{
		long long transition_32 = 0;
		float tmp = 0;
		float last_tmp = 0;
		int sign = 0;
		int exponent = 0;
		float mantissa = 0;
		transition_32 |= data_4 << 24;//�õ����ݵĵ�8λ
		transition_32 |= data_3 << 16;
		transition_32 |= data_2 << 8;
		transition_32 |= data_1;
		sign = (transition_32 & 0x80000000) ? -1 : 1;//����λ
		//�����Ʋ������ٰ�λ����㣬���������30��23λ��Ӧ��e
		exponent = ((transition_32 >> 23) & 0xff) - 127;
		//��22~0ת��λ10���ƣ��õ���Ӧ��xϵ��
		mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
		tmp = sign * mantissa * pow(2, exponent);
		last_tmp = tmp;
		return tmp;
	}
	void  SerialPort::readcomn100()
	{
		unsigned char ahrs_packet[56];
		bool isuseful = false;
		readn100(ahrs_packet, &isuseful);
		if (isuseful)
		{
			//cout << "--------------------------------------------" << endl;
			count = ahrs_packet[3];
			//cout << "count" << endl << count << endl;
			Roll = data_trans(ahrs_packet[19], ahrs_packet[20], ahrs_packet[21], ahrs_packet[22]);//�����(rad)
			roll = (180 * Roll) / PI;
			//cout << "roll" << endl << roll << endl;
			Pitch = data_trans(ahrs_packet[23], ahrs_packet[24], ahrs_packet[25], ahrs_packet[26]);//������
			pitch = (180 * Pitch) / PI;
			//cout << "pitch" << endl << pitch << endl;
			Heading = data_trans(ahrs_packet[27], ahrs_packet[28], ahrs_packet[29], ahrs_packet[30]);//ƫ����
			heading = (180 * Heading) / PI;

			Qw = data_trans(ahrs_packet[31], ahrs_packet[32], ahrs_packet[33], ahrs_packet[34]);
			Qx = data_trans(ahrs_packet[35], ahrs_packet[36], ahrs_packet[37], ahrs_packet[38]);
			Qy = data_trans(ahrs_packet[39], ahrs_packet[40], ahrs_packet[41], ahrs_packet[42]);
			Qz = data_trans(ahrs_packet[43], ahrs_packet[44], ahrs_packet[45], ahrs_packet[46]);

			//cout << "heading" << endl << heading << endl;
			Timestamp = timestamp(ahrs_packet[50], ahrs_packet[49], ahrs_packet[48], ahrs_packet[47], ahrs_packet[51], ahrs_packet[52], ahrs_packet[53], ahrs_packet[54]);//ʱ���(us)
			//cout <<  Timestamp << "    Timestamp" << endl;
			//cout << "--------------------------------------------" << endl;
			//cout << "pitch:" << pitch << " " << "heading:" << heading << " " << "roll:" << roll << endl;
		}
		return;
	}

#endif
