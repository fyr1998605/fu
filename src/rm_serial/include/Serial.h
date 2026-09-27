#ifndef _SERIAL_H
#define _SERIAL_H

#include<iostream>
#include<vector>
#include<boost/asio.hpp>
#include<boost/bind.hpp>
#include <chrono>//计时
#include <cstdlib>//exit
#include <cmath>//pow
#include <geometry_msgs/Twist.h>//定义了线性和角速度的运动控制信息

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
	io_service m_IoService;//用于处理异步操作的事件循环

	boost::system::error_code ec;//用于存储错误代码

	serial_port m_serialPort;//串口对象

	int flag, open_flag, buffer_flag;//标志位，用于控制程序流程

	double quat[4],gyro[3],accel[3];//用于存储四元数、陀螺仪和加速度计数据
	vector<unsigned char> data_read;//用于存储读取的数据
	vector<unsigned char> buffer_data;//用于存储缓冲的数据
	int count;//计数器
	float  Heading, Pitch, Roll;//用于存储欧拉角
	float heading, pitch, roll;//用于存储处理后的角度值?
	float Qw, Qx, Qy, Qz;//四元数分量
	long long int Timestamp;//时间戳
	unsigned char data_one;//单个字节数据

	vector<vector<int>>Angle;
	vector<int> Rm_Serial_Data;

	SerialPort(const char* dev_name, int paket_len);//构造函数，初始化串口设备

	vector<vector<int>>Get_Pitch_Yaw(vector<unsigned char>& _data);//解析俯仰角和偏航角

	vector<vector<int>>Get_P_Y_R(vector<unsigned char>& _data);//解析俯仰角、偏航角和滚转角

	vector<int>Get_Imu_C_Dj(vector<unsigned char>& _data);//解析IMU数据

	bool Readcom();//读取串口数据

	bool Readcom_imu(int data_len);//取IMU数据

	void  readn100(unsigned char* data, bool* isuseful);//读取特定格式的数据

	long long timestamp(unsigned char data_1, unsigned char data_2, unsigned char data_3, unsigned char data_4, unsigned char data_5, unsigned char data_6, unsigned char data_7, unsigned char data_8);//转换时间戳

	float data_trans(unsigned char data_1, unsigned char data_2, unsigned char data_3, unsigned char data_4);//转换数据

	void  readcomn100();//读取并处理N100协议的数据

	void ProtolOfSerial(int euler_angle[3], int angular_velocity[3], int accelerated_velocity[3], char camp, char shoot_flag);

	void send_com_test();

	void send_cmdvel(int lx , int ly , int wz, int tp , int ty , int flag, int up_flag, int signal,int super_speed, int lock_head);

	long long timestamp(unsigned char Data_1, unsigned char Data_2, unsigned char Data_3, unsigned char Data_4);

};

// #define NO 9999999

//判断串口是否正常开启
SerialPort::SerialPort(const char* dev_name, int paket_len) :m_serialPort(m_IoService), data_read(paket_len)
{
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

//串口发送函数(哨兵16字节下行帧)
void SerialPort::send_cmdvel(int lx , int ly , int wz, int tp , int ty , int flag,int up_flag,int signal,int super_speed, int lock_head)
{
	//创建数据包
    unsigned char data_write[16]={0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xFE};//16字节
								/*帧头-P轴高-P轴低-Y轴高-Y轴低-视标--无-锁头--X线高-X线低-Y线高-Y线低-z角高-z角低-杂--帧尾*/
								/*00---01---02----03---04---05---06---07---08---09---10---11---12---13---14--15*/

	data_write[5] = signal;//5---视觉标志位
	data_write[6]=super_speed;//6---超电
	data_write[7]=lock_head;//7---锁头标志位(0:不锁头, 1:锁头)
	data_write[14]=	up_flag;//14---前哨站，小陀螺，4上坡

    int high_pitch = 0;//1
	int low_pitch = 0;//2
	int high_yaw = 0;//3
	int low_yaw = 0;//4

    /*************线速度高位低位**************/
    int line_high_x = 0;//8
	int line_low_x = 0;//9
	int line_high_y = 0;//10
    int line_low_y = 0;//11
    /*************角速度高位低位**************/
    int angular_high_z = 0;//12
    int angular_low_z = 0;//13
    /*************P-Y转角高位低位**************/
    int turn_high_p = 0;
    int turn_low_p = 0;
    int turn_high_y = 0;
    int turn_low_y = 0;
	if (flag == 0)
	{

	}

	else
	{
		if (lx >= 0){
            line_high_x = lx >> 8;	//保留高八位
            line_low_x = lx & 255;	//保留低八位
        }
        else{
            lx = -lx;				//二进制负数在计算机中表示补码，将负的变成正的，方便计算
            line_high_x = lx >> 8;	//保留高八位
            line_low_x = lx & 255;	//保留低八位
            line_high_x += 128;			//最后在末尾加上1，标明该数据为负数
        }
		if (ly >= 0){
            line_high_y = ly >> 8;	//保留高八位
            line_low_y = ly & 255;	//保留低八位
        }
        else{
            ly = -ly;				//二进制负数在计算机中表示补码，将负的变成正的，方便计算
            line_high_y = ly >> 8;	//保留高八位
            line_low_y = ly & 255;	//保留低八位
            line_high_y += 128;			//最后在末尾加上1，标明该数据为负数
        }

        if (wz >= 0){
            angular_high_z = wz >> 8;
            angular_low_z = wz & 255;
        }
        else{
            wz = -wz;
            angular_high_z = wz >> 8;
            angular_low_z = wz & 255;
            angular_high_z += 128;
        }
        /*-------------------线速度，角速度数据存入包-----------------------*/
		data_write[8] = line_high_x;
		data_write[9] = line_low_x;
		data_write[10] = line_high_y;
		data_write[11] = line_low_y;
		data_write[12] = angular_high_z;
		data_write[13] = angular_low_z;

        if (tp >= 0){
            turn_high_p = tp >> 8;
            turn_low_p = tp & 255;
        }
        else{
            tp = -tp;
            turn_high_p = tp >> 8;
            turn_low_p = tp & 255;
            turn_high_p += 128;
        }
        if (ty>= 0){
            turn_high_y = ty >> 8;
            turn_low_y = ty & 255;
        }
        else{
            ty = -ty;
            turn_high_y = ty >> 8;
            turn_low_y = ty & 255;
            turn_high_y += 128;
        }

        /*-------------------P-Y轴偏角数据传入数据包-----------------------*/
        data_write[1] = turn_high_p;
        data_write[2] = turn_low_p;
        data_write[3] = turn_high_y;
        data_write[4] = turn_low_y;

	}

	//写入串口
	size_t len_write = this->m_serialPort.write_some(buffer(data_write), ec);

	if (ec)
	{
		// cout << "m_serialport.write_some fail " << endl;
	}
	else
	{
		// cout << "m_serialport.write_some success " << endl;
	}

}


vector<vector<int>>SerialPort::Get_Pitch_Yaw(vector<unsigned char>& _data)
{
	vector<vector<int>>angle_temp;
	vector<int>temp;
	int pitch, yaw;

	if (_data[0] == 0xFF && _data[5] == 0x01 && _data[6] == 0x00 && _data[7] == 0xFE && _data.size() == 8)
	{
		//解析p
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
		//解析y
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
		//解析p
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
		//解析y
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
		//解析r
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
//-----------------------------------解析串口数据--大疆c板陀螺仪--哨兵49字节上行帧----------------------------------
vector<int>SerialPort::Get_Imu_C_Dj(vector<unsigned char>& _data)
{
	vector<int> rmserial;
	double num = 0;
	for(int i = 0; i < 3; i++)	//解析[1,2,3],[4,5,6],[7,8,9]的数据										【0 ， 1 ， 2】
	{
		if((_data[3*i+1] & 0x80) == 0x80)
			num = -((_data[3*i+1] & 0x7f)*256*256 + _data[3*i+2] * 256 + _data[3*i+3] * 1);
		else
			num = _data[3*i+1]*256*256 + _data[3*i+2] * 256 + _data[3*i+3] * 1;
		rmserial.push_back(num);
	}
	for(int i = 0; i < 3; i++)	//解析[11,12,13],[14,15,16],[17,18,19]的数据							【3 ， 4 ， 5】
	{
		if((_data[3*i+11] & 0x80) == 0x80)
			num = -((_data[3*i+11] & 0x7f)*256*256 + _data[3*i+12] * 256 + _data[3*i+13] * 1);
		else
			num = _data[3*i+11]*256*256 + _data[3*i+12] * 256 + _data[3*i+13] * 1;
		rmserial.push_back(num);
	}
	for(int i = 0; i < 3; i++)	//解析[21,22,23],[24,25,26],[27,28,29]的数据							【6 ， 7 ， 8】
	{
		if((_data[3*i+21] & 0x80) == 0x80)
			num = -((_data[3*i+21] & 0x7f)*256*256 + _data[3*i+22] * 256 + _data[3*i+23] * 1);
		else
			num = _data[3*i+21]*256*256 + _data[3*i+22] * 256 + _data[3*i+23] * 1;
		rmserial.push_back(num);
	}
	rmserial.push_back(_data[31]);				//阵营位 0红 1蓝											【 9 	】
	rmserial.push_back(_data[32]);				//射击位(本工程写死打符, 不使用)								【 10 	】
	for(int i=0;i<2;i++)						//解析[33,34]哨兵hp,[35,36]哨兵射速 							【11 ， 12】
	{
    	if((_data[2*i+33] & 0x80) == 0x80)
			num = -(((int)_data[2*i+33] << 8) |	_data[2*i+34]);
		else
			num = (((int)_data[2*i+33] << 8) | _data[2*i+34]);
		rmserial.push_back(num);
    }
	rmserial.push_back(_data[37]);				//补给点是否被占用/机器人存活位								【 13 】
	for(int i=0;i<1;i++)						//解析[38，39]前哨站HP										【 14 】
	{
    	if((_data[1*i+38] & 0x80) == 0x80)
			num = -(_data[1*i+38]*256 +	_data[1*i+39]*1  );
		else
			num = (_data[1*i+38]*256 +	_data[1*i+39]*1  );
		rmserial.push_back(num);
    }
	rmserial.push_back(_data[41]);				//云台手是否发送点位											【 15 】
	for(int i=0;i<2;i++)						//解析[42,43] X坐标,[44,45]Y坐标  							【16 ， 17】
	{
		num = (_data[2*i+42]*256 +	_data[2*i+43]*1  );
		rmserial.push_back(num);
    }
	for(int i=0;i<1;i++)						//解析[46,47] 允许发弹量 										【18】
	{
    	if((_data[1*i+46] & 0x80) == 0x80)
			num = -(_data[1*i+46]*256 +	_data[1*i+47]*1  );
		else
			num = (_data[1*i+46]*256 +	_data[1*i+47]*1  );
		rmserial.push_back(num);
    }

	return rmserial;
}
//-------------------------------------------------------------------------------------------------------

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

//-------------------------------------------------读取串口(哨兵49字节上行帧)---------------------------------------------
bool SerialPort::Readcom_imu(int data_len)
{
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	// 读入临时缓冲: 修复哨兵原代码 data_read(32) 与读取长度(39) 不匹配导致的越界问题
	vector<unsigned char> data_temp(data_len);
	size_t len_read = this->m_serialPort.read_some(buffer(data_temp, data_len), ec);

	if(ec)
	{
		cout << "----读取电控串口失败----" << endl;
		ROS_ERROR("串口断开,主动结束节点以触发串口重启机制");
		ros::shutdown();	// 优雅关闭ROS
		exit(EXIT_FAILURE);	// 强制自杀,打破死循环
		return false;
	}
	for(size_t i = 0; i < len_read; i++)
		buffer_data.push_back(data_temp[i]);

	bool frame_received = false;
	if(buffer_data.size() >= 49*3)				//积累足够数据后再扫描
	{
		for(size_t i = 0; i + 49 <= buffer_data.size(); i++)
		{
			//哨兵49字节帧校验位: 0xff / 0x66 / 0x77 / 0x88 / 0x99 / 0xfe
			if(buffer_data[i] == 0xff && buffer_data[i+10] == 0x66 && buffer_data[i+20] == 0x77 &&
			   buffer_data[i+30] == 0x88 && buffer_data[i+40] == 0x99 && buffer_data[i+48] == 0xfe)
			{
				vector<unsigned char> enable_data;
				for(int j = 0; j < 49; j++)
					enable_data.push_back(buffer_data[i+j]);

				Rm_Serial_Data.clear();
				Rm_Serial_Data = Get_Imu_C_Dj(enable_data);
				frame_received = true;
				buffer_data.clear();
				break;
			}
		}
		//长期无有效帧时清空, 防止缓冲无限增长
		if(buffer_data.size() >= 200)
			buffer_data.clear();
	}
	return frame_received;
}

/*-------------------------------------N100协议---------------------------------------------*/

void  SerialPort::readn100(unsigned char* data, bool* isuseful)
{
	bool istype = false;
	bool isimu = false;
	bool isarhs = false;
	unsigned char receive_data[80];
	int count = 0;
	while (1)
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

long long SerialPort::timestamp(unsigned char data_1, unsigned char data_2, unsigned char data_3, unsigned char data_4, unsigned char data_5, unsigned char data_6, unsigned char data_7, unsigned char data_8)
{
	long long temp;
	uint16_t hh_16, h_16, l_16, ll_16;
	uint32_t h_32, l_32;

	hh_16 = data_1 << 8 | data_2;
	h_16 = data_3 << 8 | data_4;
	l_16 = data_5 << 8 | data_6;
	ll_16 = data_7 << 8 | data_8;

	h_32 = (uint16_t)hh_16 << 16 | (uint16_t)h_16;
	l_32 = (uint16_t)l_16 << 16 | (uint16_t)ll_16;
	temp = (uint32_t)h_32 << 32 | (uint32_t) l_32;

	return temp;
}

long long SerialPort::timestamp(unsigned char Data_1, unsigned char Data_2, unsigned char Data_3, unsigned char Data_4)
{
	unsigned long temp;
	uint16_t H_16, L_16;

	H_16 = Data_1 << 8 | Data_2;
	L_16 = Data_3 << 8 | Data_4;

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
	transition_32 |= data_4 << 24;
	transition_32 |= data_3 << 16;
	transition_32 |= data_2 << 8;
	transition_32 |= data_1;
	sign = (transition_32 & 0x80000000) ? -1 : 1;
	exponent = ((transition_32 >> 23) & 0xff) - 127;
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
		count = ahrs_packet[3];
		Roll = data_trans(ahrs_packet[19], ahrs_packet[20], ahrs_packet[21], ahrs_packet[22]);
		roll = (180 * Roll) / PI;
		Pitch = data_trans(ahrs_packet[23], ahrs_packet[24], ahrs_packet[25], ahrs_packet[26]);
		pitch = (180 * Pitch) / PI;
		Heading = data_trans(ahrs_packet[27], ahrs_packet[28], ahrs_packet[29], ahrs_packet[30]);
		heading = (180 * Heading) / PI;

		Qw = data_trans(ahrs_packet[31], ahrs_packet[32], ahrs_packet[33], ahrs_packet[34]);
		Qx = data_trans(ahrs_packet[35], ahrs_packet[36], ahrs_packet[37], ahrs_packet[38]);
		Qy = data_trans(ahrs_packet[39], ahrs_packet[40], ahrs_packet[41], ahrs_packet[42]);
		Qz = data_trans(ahrs_packet[43], ahrs_packet[44], ahrs_packet[45], ahrs_packet[46]);

		Timestamp = timestamp(ahrs_packet[50], ahrs_packet[49], ahrs_packet[48], ahrs_packet[47], ahrs_packet[51], ahrs_packet[52], ahrs_packet[53], ahrs_packet[54]);
	}
	return;
}

#endif
