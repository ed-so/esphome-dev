//#############################################################################
/** \file	resi-server.cc TCP/stdin/serial server for controlling resistor network card
 \author		T.Motylewski@wolf-woelfel.de
*/
//#############################################################################

char version[] = "RESI V3.10";


//-----------------------------------------------------------------------------
//                                  INCLUDES
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <assert.h>
#include <fstream>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include "can4linux.h"
#include "com_can.h"
#include "resi-fpga.h"

extern "C" {
#include "IPC.h"
}

// RESI and LCD drivers
#include "st7565.h"

#include "line-buffer.h"
#include <list>
// for strtol
#include <cstdlib>
// for toupper
// #include <cctype>
// for transform
//#include <algorithm>

#define STDDEV "can0"
typedef unsigned char BYTE;
typedef unsigned int UINT4;

enum resi_const {
	DEFAULT_SERVER_PORT = 43007,
	MAX_MSG_SIZE = 4000,
	RXBUFFERSIZE = 100,
	NET_ADDR_L = 64,
	RESITYPE_L = 64,
	UBOOT_CFG_ADR = 0x4000,
	UBOOT_CFG_L = 0x2000,
	UBOOT_CFG_HDR = 0x4,
};


/** tm means time in units of 1/1000 s since program started (tsStarted)
 * tm time stays positive for 2.48 days after system boot.
 * It can be redefined by changing TM_HZ value to something else than 1000.
 * Always use TM_HZ when comparing time with real seconds.
 * The code should work after "time wraparound" but if something happens you know why.
 * ts means time in sec */
#define TM_HZ 1000


#define TRUE true
#define FALSE false
#define SERIAL_DEV "/dev/ttyS1"
//=================================================CAN
#define ACK_FIND 0x10
#define ACK_GET 0x13
#define ACK_TYPE 0x18
#define ACK_CAN 0x1f	//for the rest of can-cmd.

//for broadcasting 
#define BROAD_CAN_RID 100
//RESI-Serialnumber
#define SN 1234
//RESI-TYP
#define TYP  0x2
#define DIN_MASK 0xf

#define RESOLUTION 1 //for resolution 1 -> 0x81 for resolution 10 -> 0x91 (Formel in der Spezifikation)

int debug = 0;
int watchdog = 0;

unsigned long brid = BROAD_CAN_RID;
//var. for RID
unsigned long rid = 100;
//var. for TID
unsigned long tid = 101;
unsigned long serialnr = SN;
unsigned long serial_pcb = 0;
//Typ
//can mesg. ext
unsigned int ext=0;
unsigned int can_identifier=(1<<11)-1;	//std. 536870911 (29 bits) or 2047(11 bits)
unsigned int can_bitrate = 500;
//====================================================

#define WARN(...) (fprintf(stderr,"WARN:" __VA_ARGS__),fprintf(stderr,"\n"))
#define ERR(...) (fprintf(stderr,"ERROR:" __VA_ARGS__),fprintf(stderr,"\n"))

#ifdef DEBUG
	#define DMSG(fmt...) (fprintf(stderr,"DBG:" fmt),fprintf(stderr," %f\n",gettime()))
	//very usefull for debugging data change after some miliseconds combined with dumpnc.c

	#define DMSG(fmt...) (fprintf(stderr,"DBG:" fmt),fprintf(stderr,"\n"))
#else
	#define DMSG(fmt...)
#endif

void WriteConfig(void);
int GetMessage();
void SendACK(int fd,int);
int set_bitrate(int fd,int baud);
class t_CHANNELS;
unsigned int getuint(int n,  const unsigned char *ptr);
int can_load(t_CHANNELS);
int can_set(t_CHANNELS ch);
int can_find();
int can_get(t_CHANNELS ch);
int CanHandleCommand();
int can_mode();
int set_can_rid();
int set_can_tid();
int can_rst();
int can_type();
int can_clear(t_CHANNELS ch);
t_CHANNELS CanParseChannel(BYTE);
BYTE getChannel();
BYTE getCMD();
int conversion(double);
int openConfig();

/** global variables */

typedef list<class LineBuffer*> t_STREAM_LIST;
t_STREAM_LIST streams;

void WriteToAll(const string & str, LineBuffer::flags dest) {
	t_STREAM_LIST::iterator i;
	for(i=streams.begin(); i!=streams.end(); ++i) {
		if((*i)->m_flags & dest) {
			write((*i)->m_fd, str.data(), str.length());
		}
	}
}	

typedef int t_TIME;

unsigned short int myport;
unsigned short int myudpport;
unsigned short int toport;
struct sockaddr_in myaddr, fromaddr, recvaddr, toaddr;

#define UBOOT_CFG_DEV "/dev/uboot"
char tnow_str[256] = "time not initialized";
char uboot_cfg[UBOOT_CFG_L+100];


int fdNet;
int fdUdp;
int fdError;
int fdIn;
int can_fd;
canmsg_t rx;
canmsg_t tx;

enum e_resi_type {
	RESI_T_K = 0,
	RESI_T_M = 1
};	
int ResiType = RESI_T_K;

#define CONFIG_MTD_DEV "/dev/config-m"
char config_name[200] = "/etc/resi.cfg";

FILE * fLog;
long iBusFlags;
time_t tsStarted;
long tmLastVerified=0;
long tmNow;  // current time, global, will be updated in many places
long tmLastSentReceived=0;
struct timeval tvNow;
struct timeval tvSelectTimeout;

/** all t_TIME in units of us */
t_TIME time0;
t_TIME SleepUntil=0;
t_TIME time_v_changed=0;

int quiet = 0;

int RecentDIN = -1; // -1 to force refresh on start
int RecentDOUT = 0;

/** return time in "ticks", not used */
long tmGet() {
	gettimeofday(&tvNow,NULL);
	return (tmNow=TM_HZ*(tvNow.tv_sec-tsStarted) + (TM_HZ*tvNow.tv_usec)/1000000);
#if TM_HZ>2147
#error when using 32 bit signed int TM_HZ can not be greater than 2147
#endif
}

/** microseconds, 32 bit, will wrap around, use only for relative comparison ! */

t_TIME tnow=0;
/** return time in us from unspecified origin and update tnow */
t_TIME gettime() {
	gettimeofday(&tvNow,NULL);
	return ( tnow = tvNow.tv_sec*1000000 + tvNow.tv_usec);
}



void die(char * str) {
	fprintf(stderr,"exiting: %s, errno %d\n", str, errno);
	exit(10);
}


/** set KEEPALIVE options for a socket
 * Unfortunately the kernel does NOT send keepalive when there are some
 * data in the transmit FIFO. Therefore detection of dead socket will
 * only work if there are no data sent at the very moment when the
 * connection is lost. Detection of dead socket is
 * done by monitoring the amount of data in the transmit FIFO
 * - if constant: timeout.
	@param fd filedescriptor from the opened socket
	@param options not used now
	@return 0
*/
int SetTcp(int fd, int options) {
	int i;

	i = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &i, sizeof(i));
	setsockopt(fd, SOL_TCP, TCP_NODELAY, &i, sizeof(i));

	i = 1; /* close connection after 20 s no response */
	setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &i, sizeof(i));
	i = 20; // close the connection after missing 20 KEEPALIVE
	setsockopt(fd, SOL_TCP, TCP_KEEPCNT, &i, sizeof(i));
	i = 1; // 1 second before sending KEEPALIVE
	setsockopt(fd, SOL_TCP, TCP_KEEPIDLE, &i, sizeof(i));
	i = 1; // 1 second between KEEPALIVE
	setsockopt(fd, SOL_TCP, TCP_KEEPINTVL, &i, sizeof(i));

	return 0;
}

int OpenListeningTcp(int port) {
	int tmp;
	int sock;
	struct sockaddr_in address;

	if ((sock = socket(PF_INET, SOCK_STREAM, 0)) < 0)
		die("socket");

	/* Let the kernel reuse the socket address. This lets us run
	twice in a row, without waiting for the (ip, port) tuple
	to time out. */
	tmp = 1;
	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &tmp, sizeof(tmp));
	/* switch OFF TCP buffering on this socket*/
	tmp = 1;
	setsockopt(sock, SOL_TCP, TCP_NODELAY, &tmp, sizeof(tmp));

	SetTcp(sock, 0);

	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	memset(&address.sin_addr, 0, sizeof(address.sin_addr));

	if (bind(sock, (struct sockaddr *) &address, sizeof(address)))
		die("bind");

	if (listen(sock, 1)) // max 1 waiting connection
		die("listen");
	return sock;
}

int ConfigSerial(int fd, int speed, int lflag) {
	struct termios ts;
	int ret;
	
	ret = tcgetattr(fd, &ts);
	if(ret) {
		perror("tcgetattr");
		return ret;
	}
	ts.c_iflag = 0; // make it in APP SW IGNCR;
	ts.c_oflag = 0 /*| OPOST | ONLCR*/;
	ts.c_cflag = CS8 | CREAD | CLOCAL;
	//ts.c_cc[VEOL] = 27;
	cfsetospeed(&ts, speed);
	cfsetispeed(&ts, B0 /*SerialDefaultSpeed*/);
	
	ts.c_lflag = lflag;
	ret = tcsetattr(fd, TCSANOW, &ts);
	if(ret) {
		perror("tcsetattr");
		return ret;
	}
	return 0;
}

int OpenSerial(const char *dev, int speed, int lflag) {
	int fd;
	int ret;
	
	fd = open(dev, O_RDWR);
	ret = ConfigSerial(fd, speed, lflag);
	if(ret) {
		WARN("OpenSerial ConfigSerial %s %d 0x%x 0x%x failed %d",
			dev, fd, speed, lflag, ret);
		return ret;
	} else {
		return fd;
	}
}

enum e_CHANNELS {
	CH_R0 = 0,
	CH_R1 = 1,
	CH_R_N = 2,
	CH_DOUT = 2,
	CH_DIN = 3,
	CH_R0V = 4,
	CH_R1V = 5,
	
	CH_N,
	CH_INVALID = 0x80000000
};

enum e_MODE {
	REMOTE = 0,
	LOCAL = 1
};

enum e_CONSTANTS {
	INDEX_MAX = 256,
	DEBOUNCE_DIN = 20000
};

e_MODE Mode[CH_R_N];

int LastDIN[CH_R_N];
int ConfirmedDIN[CH_R_N];
t_TIME tLastDINChanged[CH_R_N];

/** class storing bitmask of active channels (up to 15 bits) in bits 16..30 of 32 bit uint
	and "virtual channel" SZ value in lower 16 bits */
class t_CHANNELS {
protected:
	unsigned int chn; /// bits 0..15SZ, bits 16..30 selected channel, bit 31 = INVALID channel 
public:
	t_CHANNELS(): chn(0){};
	t_CHANNELS(unsigned int x) : chn(x) {};
	bool operator& (enum e_CHANNELS ch) {
		
		//cout<<"operator& (enum e_CHANNELS ch),  ch : "<<ch<<"  m_chn : "<<chn<<"\n";
		//cout<<"operator&(enum e_CHANNELS ch) ,  ((0x10000<<ch) & chn) != 0 :  "<<(((0x10000<<ch) & (int)chn) != 0)<<"\n";
		//return ((0x10000<<(int)ch) & chn) != 0;
		return (*this) & (int) ch;
	}
	bool operator& (int ch) {
		//cout<<"operator& (int ch),  ch : "<<ch<<"  m_chn : "<<chn<<"\n";
		//cout<<"operator&(int ch) ,  ((0x10000<<ch) & chn) != 0 :  "<<(((0x10000<<ch) & (int)chn) != 0)<<"\n";
		return ((0x10000<<ch) & chn) != 0;
	}
	int getv() {
		return chn & 0xffff;
	}
	t_CHANNELS & operator| (enum e_CHANNELS ch) {
		return *this | (int)ch;
	}
	t_CHANNELS & operator| (int ch) {
		chn |= (0x10000<<ch);
		return *this;
	}
	/** set SZ value (16 bit)
	@param v SZ value to be stored in this class
	@return class reference
	*/
	t_CHANNELS & setv(int v) {
		chn &= ~0xffff;
		chn |= v & 0xffff;
		return *this;
	}
	operator int () {
		return chn;
	}
};

int CH(int ch) {
	return 0x10000<<ch;
}


double ValCurrent[CH_N];
double ValSet[CH_R_N];
double ValLoad[CH_N];
double ValVirtual[CH_R_N][INDEX_MAX];
int Index[CH_R_N];

string ValIpAddr = "";
string ValIpNetmask = "";
string ValIpGateway = "";
int ValConfigVersion = -1;

t_CHANNELS ParseChannel(string & chan) {
	t_CHANNELS channels = 0;
	const char *str = chan.data();
	if(chan.length() == 0) {
		channels | CH_R0 | CH_R1 | CH_DOUT;
	} else if(chan.length() == 1) {
		if(chan[0] == '0') {
			channels | CH_R0;
		} else if(chan == "1") {
			channels | CH_R1;
		} else { 
			return CH_INVALID;
		}
	} else if(chan.length() == 2) {
		return CH_INVALID;
	} else { // min 3 characters
		if(str[1] == 'V') {
			int tmp = atoi(str+2);
			if(tmp<0 || tmp >= INDEX_MAX) {
				return CH_INVALID;
			}
			channels.setv(tmp);
			if(str[0] == '0') {
				channels | CH_R0V;
			} else if(str[0] == '1') {
				channels | CH_R1V;
			} else {
				return CH_INVALID;
			}
		} else if(chan == "ALL") {
			channels | CH_R0 | CH_R1;
		} else if(chan == "DOUT") {
			channels | CH_DOUT;
		} else if(chan == "DIN") {
			channels | CH_DIN;
		} else {
			return CH_INVALID;
		}	
	}
	return channels;
}

void Changed(bool chd) {
	if(chd) {
		if(!time_v_changed) {
			time_v_changed = tnow|1;
		}
	} else {
		time_v_changed = 0;
	}
}

/** save config to a file and commit to flash.
 * @param ch ignored at the moment
 */
void SaveConfig(t_CHANNELS ch) {
	int ret;
	char cmd[200];
	sprintf(cmd, "/bin/flashw -f '%.70s' '%.50s'; /bin/sync", config_name, CONFIG_MTD_DEV);
	ret = system(cmd);
}

//from uboot environment
//int serialnr;
typedef struct s_ctrl {
	char ethaddr[NET_ADDR_L];
	char ipaddr[NET_ADDR_L];
	char netmask[NET_ADDR_L];
	char gatewayip[NET_ADDR_L];
	char resitype[RESITYPE_L];
} t_CTRL;

t_CTRL ctrl_;
t_CTRL * ctrl = &ctrl_;

char * e_match(char * env, char * name) {
	do {
		if(*name != *env)
			return NULL;
		env++;
		name++;
	} while(*name);
	return env;
}

void ReadConfigUboot(void) {
	int fdin;
	int ret;
	char *eptr;
	char *val;
	memset(uboot_cfg, 0, sizeof(uboot_cfg));
	fdin=open(UBOOT_CFG_DEV, O_RDONLY);
	if(fdin<0) {
		perror("config uboot:" UBOOT_CFG_DEV);
		WARN("config uboot:" UBOOT_CFG_DEV " open failed");
		return;
	}
	ret = lseek(fdin, UBOOT_CFG_ADR, SEEK_SET);
	if(ret != UBOOT_CFG_ADR) {
		perror("lseek uboot:");
		WARN("lseek uboot failed %d", ret);
		close(fdin);
		return;
	}
	ret = read(fdin, uboot_cfg, UBOOT_CFG_L);
	if(ret !=  UBOOT_CFG_L) {
		int err = errno;
		perror("read uboot:");
		WARN("read uboot failed %d %d", ret, err);
		close(fdin);
		return;
	}
	close(fdin);
	for(eptr = uboot_cfg + UBOOT_CFG_HDR; eptr < uboot_cfg+UBOOT_CFG_L; eptr++ ) {
		if(!*eptr) 
			continue;
		//DMSG("ematch: %s", eptr);
		if((val = e_match(eptr, "serial#="))) {
			sscanf(val, "%d", &serialnr);
			if(debug>1) {
				DMSG("serial#=%d", serialnr);
			}
		}
		if((val = e_match(eptr, "serialnr="))) {
			sscanf(val, "%d", &serial_pcb);
			if(debug>1) {
				DMSG("serialnr=%d", serial_pcb);
			}
		}
		if((val = e_match(eptr, "ethaddr="))) {
			int i;
			strncpy(ctrl->ethaddr, val, sizeof(ctrl->ethaddr));
			ctrl->ethaddr[sizeof(ctrl->ethaddr)-1] = 0;
			for(i=0; *val && (i<sizeof(ctrl->ethaddr)-1); val++) {
				if(*val == ':')
					continue;
				ctrl->ethaddr[i++] = *val;
			}
			ctrl->ethaddr[i] = 0;
			if(debug>1) {
				DMSG("ethaddr=%s", ctrl->ethaddr);
			}			
		}
		if((val = e_match(eptr, "ipaddr="))) {
			strncpy(ctrl->ipaddr, val, sizeof(ctrl->ipaddr));
			ctrl->ipaddr[sizeof(ctrl->ipaddr)-1] = 0;
			if(debug>1) {
				DMSG("ipaddr=%s", ctrl->ipaddr);
			}			
		}
		if((val = e_match(eptr, "gatewayip="))) {
			strncpy(ctrl->gatewayip, val, sizeof(ctrl->gatewayip));
			ctrl->gatewayip[sizeof(ctrl->gatewayip)-1] = 0;
			if(debug>1) {
				DMSG("gatewayip=%s", ctrl->gatewayip);
			}			
		}
		if((val = e_match(eptr, "netmask="))) {
			strncpy(ctrl->netmask, val, sizeof(ctrl->netmask));
			ctrl->netmask[sizeof(ctrl->netmask)-1] = 0;
			if(debug>1) {
				DMSG("netmask=%s", ctrl->netmask);
			}			
		}	
		if((val = e_match(eptr, "resitype="))) {
			strncpy(ctrl->resitype, val, sizeof(ctrl->resitype));
			ctrl->resitype[sizeof(ctrl->resitype)-1] = 0;
			if(debug>1) {
				DMSG("resitype=%s", ctrl->resitype);
			}			
		}	
		while((eptr < uboot_cfg+UBOOT_CFG_L) && *eptr) {
			eptr++;
		}	
	}
	if(e_match(ctrl->resitype, "266k")) {
		ResiType = RESI_T_K;
	} else if(e_match(ctrl->resitype, "2M6")) {
		ResiType = RESI_T_M;
	}
}


/** read from flash, skipping the last block of 0xff */
void ReadConfigFlash(char * fname) {
	FILE *fin;
	FILE *fout;
	int ffcnt = 0;
	int inp;
	fin=fopen(CONFIG_MTD_DEV, "r");
	if(fin==NULL) {
		perror("config flash:" CONFIG_MTD_DEV);
		return;
	}
	fout = fopen(fname, "w+");
	if(fout==NULL) {
		perror("config:" CONFIG_MTD_DEV);
		return;
	}
	while((inp=fgetc(fin)) != EOF) {
		if(inp == 0xff) {
			ffcnt++;
		} else {
			while(ffcnt) {
				ffcnt--;
				fputc(0xff, fout);
			}
			fputc(inp, fout);
		}
	}
	fclose(fout);
	fclose(fin);
}

void DoRST(void) {
	t_STREAM_LIST::iterator i;
	cerr << "DoRST\nexiting 0\n";
	if(time_v_changed) {
		WriteConfig();
		SaveConfig(0);
		Changed(false);
	}

	LcdClear();
	for(i=streams.begin(); i!=streams.end(); ++i) {
		close((*i)->m_fd);
	}
	system("/bin/sync;/sbin/reboot");
	exit(0);
}

enum {
	RESI_DOUT_MAX = 15
};
enum {RESI_RELAYS_N = 3,
	RESI_R_MAX = 266665,
	RESI_DECADE_SINGLE_RESISTOR = 200000,
	RESI_DECADE_MULT_FACT = 10000,
	RESI_R_MAX_M = 2666650,
	RESI_DECADE_SINGLE_RESISTOR_M = 2000000,
	RESI_DECADE_MULT_FACT_M = 100000,
	RESI_RELAYS_BITS = 20,
	};

/** function based on PwdOutValue from Pwd-Treiber.txt (Pascal).
	@param[in] value resistance in Ohm, negative means open, maximum ::RESI_R_MAX
	@param[out] relays 3 bytes containing relay settings to get value Ohm
	@return 0 on success, -1 on error
*/ 
int CalculateRelays(int value, unsigned char relays[RESI_RELAYS_N]) {
	int i;
	int DecadeSingleResistor;
	int DecadeMultFact;
	int BytePos;
	int BitPos;
	
	if(value<0) {
		// open
		for(i=0; i<RESI_RELAYS_N; i++) {
			relays[i] = 0;
		}
		relays[2] = 0x80;
	} else {
		for(i=0; i<RESI_RELAYS_N; i++) {
			relays[i] = 0;
		}
		if(ResiType&RESI_T_M) {
			// 2.666... MOhm version
			DecadeSingleResistor = RESI_DECADE_SINGLE_RESISTOR_M;
			DecadeMultFact = RESI_DECADE_MULT_FACT_M;
		} else {
			DecadeSingleResistor = RESI_DECADE_SINGLE_RESISTOR;
			DecadeMultFact = RESI_DECADE_MULT_FACT;
		}
		
		for(i=RESI_RELAYS_BITS; i>=0; i--) {
			BytePos = i/8;
			BitPos = i%8;
			if(i%4 == 3) {
				DecadeSingleResistor = 8 * DecadeMultFact;
				DecadeMultFact /= 10;
			} else {
				DecadeSingleResistor /= 2;
			}
			
			if(value >= DecadeSingleResistor) {
				value -= DecadeSingleResistor;
				// each BytePos,BitPos pair occurs only once so XOR works OK
				// original code had OR here
				// but XOR will also work when starting from 0xff
				relays[BytePos] ^= (1<<BitPos);
			}
		}
		if(ResiType&RESI_T_M) {
			unsigned tmp;
			// we have to shift right 4 bits
			tmp = relays[2];
			relays[2] = (relays[1]>>4) | (relays[2]&0x10);
			relays[1] = (relays[1]<<4) | (relays[0]>>4);
			relays[0] = (relays[0]<<4) | (tmp&0xf);
		}
		// check whether bypass is possible, also for 2.6M
		if(relays[2] == 0 && relays[1] == 0) {
			relays[2] ^= 0x40;
			if((relays[0] & 0xf0) == 0) {
				relays[2] ^= 0x20;
			}
		}
		// invertion needed for 21 bits
		for(i=0; i<2; i++) {
			relays[i] ^= 0xff;
		}
		relays[2] ^= 0x1f;
	}
	return 0;
}

void SetRChannel(int chan, double r) {
	unsigned char relays[RESI_RELAYS_N];
	char val_str[30];
	int ri = (int)round(r);
	CalculateRelays(ri, relays);
	relays[2] ^= 0x80; // invert bit X24 here instead in FPGA RESI20
	if(debug>1) {
		cerr << "SetRChannel("<<chan<<","<<ri<<") = " <<hex<< (int)relays[0] <<" "<< (int)relays[1] <<" "<< (int)relays[2]<<"\n";
	}
	PortWrite(chan?SFMB_15_0_A:SFMA_15_0_A , relays[0] | (relays[1]<<8));
	PortWrite(chan?SFMB_23_16_A:SFMA_23_16_A , relays[2]);
	if(ri<0){
		Display57C(chan, 5*6, "OPEN        ");
		return;
	}
	if(ri/1000000) {
		sprintf(val_str, "%1d %03d %03d <", ri/1000000, (ri/1000)%1000, ri%1000);
	} else if(ri/1000) {
		sprintf(val_str, "  %3d %03d <", ri/1000, ri%1000);
	} else {
		sprintf(val_str, "      %3d <", ri);
	}
	Display57C(chan, 5*6, val_str);
}

void RefreshLcdDOUT(void) {
	int x=0;
	Display57C(3,x, "Output");
	Display57C(3,x+=6*6+3,RecentDOUT&1 ? "1":" ");
	Display57C(3,x+=6,RecentDOUT&2 ? "2":" ");
	Display57C(3,x+=6,RecentDOUT&4 ? "3":" ");
	Display57C(3,x+=6,RecentDOUT&8 ? "4":" ");
}

void RefreshLcdDIN(void) {
	int x=68;
	Display57C(3,x, "Input");
	Display57C(3,x+=5*6+3,RecentDIN&1 ? "1":" ");
	Display57C(3,x+=6,RecentDIN&2 ? "2":" ");
	Display57C(3,x+=6,RecentDIN&4 ? "3":" ");
	Display57C(3,x+=6,RecentDIN&8 ? "4":" ");
}

void RefreshIndex(void) {
	char buf[20];
	int chan;

	for(chan=0; chan<CH_R_N; chan++) {
		if(Mode[chan] == LOCAL) {
			sprintf(buf, "%3d", Index[chan]);
			Display57C(chan,17*6, buf);
		} else {
			Display57C(chan,17*6, "   ");
		}
	}
}
	


void SetDOUT(unsigned long val) {
	cerr << "SetDOUT("<<val<<")\n";
	PortWrite(RESIC_DOUT, val);
	RecentDOUT = val;
	RefreshLcdDOUT();
}

void ReadDIN(void) {
	int ret;
	ostringstream ack;
	ret = PortRead(RESIC_INS);
	// bits [3:0] digital inputs, bit 7 is F_USB_PWR_ON and works, is masked below
	if(ret ==  RecentDIN) {
		return;
	}
	ack << "OK\nGET DIN " << (ret & DIN_MASK) << "\n\r\n";
	WriteToAll(ack.str(), LineBuffer::F_ECHO_DIN);
	ValCurrent[CH_DIN] = RecentDIN = ret;
	RefreshLcdDIN();
}

void TransitionDIN(int chan, int transition) {
	if(Mode[chan] != LOCAL) {
		return;
	}
	// specification rev 0.6 Table 6
	switch(transition) {
	case 0x1: // increment
		Index[chan] = (Index[chan] + 1) % INDEX_MAX;
		break;
	case 0x2: // decrement
		Index[chan] = (Index[chan] + INDEX_MAX - 1) % INDEX_MAX;
		break;
	case 0x3: // reset 00 -> 11
	case 0x7: // 01 -> 11 (robustness)
	case 0xB: // 10 -> 11 (robustness)
		Index[chan] = 0;
		break;
	default: return;// no further action
	}
	
	// we are in LOCAL mode and Index has changed, time to change R as well
	ValCurrent[chan] = ValVirtual[chan][Index[chan]];
	SetRChannel(chan, ValCurrent[chan]);
	RefreshIndex();
	
	// send ACK message
	ostringstream ack;
	ack << "OK\nINDX "<<chan<<" "<<Index[chan]<<"\nSET "<<chan<<" "<< ValCurrent[chan]<<"\n\r\n";
	WriteToAll(ack.str(), LineBuffer::F_ECHO_INDX);
}

void TransitionMODE(int chan, enum e_MODE mode) {
	Index[chan] = 0;
	Mode[chan] = mode;
	if(mode == LOCAL) {
		ValCurrent[chan] = ValVirtual[chan][Index[chan]];
	} else {
		ValCurrent[chan] = ValSet[chan];
	}
	SetRChannel(chan, ValCurrent[chan]);
	RefreshIndex();
}

void StateMachineDIN(void) {
	int i;
	for(i=0; i<CH_R_N; i++) {
		int din = (RecentDIN>>(2*i)) & 3;
		if(din != ConfirmedDIN[i]) {
			if(din == LastDIN[i]) {
				if((tnow-tLastDINChanged[i])>DEBOUNCE_DIN) {
					// 2 bits FROM + 2 bits TO
					// FROM may be also -1 during program initialization
					TransitionDIN(i, (ConfirmedDIN[i]<<2) | din);
					ConfirmedDIN[i] = din;
				} else {
					// just wait longer
				}
			} else {
				LastDIN[i] = din;
				tLastDINChanged[i] = tnow;
			}
		}
	}
}

int HandleCommand(class LineBuffer * stream, string & line) {
	ostringstream ack;
	unsigned int channels = 0;
	int unit=1;
	char unitch = 0;
	char * end_ptr;
	double val;
	int ret;
	
	transform (line.begin(),line.end(), line.begin(), (int(*)(int)) toupper);
	istringstream input(line);
	string cmd;
	string chan;
	string value;
	bool do_rst = false;
	
	input >> cmd;
	input >> chan;
	input >> value;
	if(value != "") {
		unitch = value.end()[-1];
	}
	if(cmd == "SET" || cmd == "LOAD" ) {
		channels = ParseChannel(chan);
		if(channels == CH_INVALID || (channels & CH(CH_DIN))) {
			ack << "ERR "<< stream->m_fd<<" invalid channel :"<< chan<< ": "<<channels<<"\n";
			ack << line << "\n";
			goto error_jump;
		}
		if(value != "") {
			if((channels & (CH(CH_R0)|CH(CH_R1))) 
				&& (channels & CH(CH_DOUT))) {
				// this will never happen in ParseChannel
				WARN("HandleCommand ParseChannel problem %s %d", chan.c_str(), channels);
				ack << "ERR "<< stream->m_fd<<" channel\n";
				ack << line << "\n";
				goto error_jump;
			} else if(channels & (CH(CH_R0)|CH(CH_R1)|CH(CH_R0V)|CH(CH_R1V))) {
				if(unitch == 'K') {
					unit = 1000;
				} else if(unitch == 'M') {
					unit = 1000000;
				} else if(isdigit(unitch)) {
					unit = 1;
					unitch = 'D';
				} else {
					ack << "ERR fd="<< stream->m_fd<<" invalid unit\n";
					ack << line << "\n";
					goto error_jump;
				}
				val = strtod(value.c_str(), &end_ptr)*unit;				
				if(ResiType & RESI_T_M) {
					if(val<0.0) {
						if(val<-1.0001 || val>-0.9999) {
							ack << "ERR "<< stream->m_fd<<" wrong value\n";
							ack << line << "\n";
							goto error_jump;
						}
					} else if(val > RESI_R_MAX_M+0.0001
						|| fabs(10*round(val/10)-val) > 0.0001) {
						ack << "ERR "<< stream->m_fd<<" wrong value\n";
						ack << line << "\n";
						goto error_jump;
					}
				} else {
					if(val<-1.0001 || val > RESI_R_MAX+0.0001
						|| fabs(round(val)-val) > 0.0001) {
						ack << "ERR "<< stream->m_fd<<" wrong value\n";
						ack << line << "\n";
						goto error_jump;
					}
				}
			} else if(channels & CH(CH_DOUT)) {
				if(unitch == 'H') {
					val = strtol(value.c_str(), &end_ptr, 16);
				} else if(unitch == 'B') {
					// get binary
					val = strtol(value.c_str(), &end_ptr, 2);
				} else if(unitch == 'D' || isdigit(unitch)) {
					// get decimal
					val = strtol(value.c_str(), &end_ptr, 10);
					unitch = 'D';
				} else {
					ack << "ERR "<< stream->m_fd<<" invalid unit\n";
					ack << line << "\n";
					goto error_jump;
				}
				if(val < -0.001 || val > 15.001) {
					ack << "ERR "<< stream->m_fd<<" wrong DOUT value\n";
					ack << line << "\n";
					goto error_jump;
				}
			} else {
				ack << "ERR "<< stream->m_fd<<" invalid channel " << channels << "\n";
				ack << line << "\n";
				goto error_jump;
			}
			if(end_ptr==value.c_str()) {
				ack << "ERR "<< stream->m_fd<<" invalid "<<unitch<<" value " << value<< "\n";
				ack << line << "\n";
				goto error_jump;
			}

			for(int i=CH_R0; i<CH_N; i++) {
				if(channels & CH(i)) {
					ValLoad[i] = val;
				}
			}
			//printArr(ValLoad); //temp
					
		} // end of if(value)
		
		ack << "OK "<< stream->m_fd<<"\n";
		if(cmd == "SET") {
			for(int i=CH_R0; i<CH_N; i++) {
				if(channels & CH(i)) {
					//ack << "cur "<< ValCurrent[i] << " load "<< ValLoad[i]<< " ";
					//ack << "val "<< value << " ";
					switch(i) {
					case CH_R0:
					case CH_R1:
						if(Mode[i]!=LOCAL) {
							ValSet[i] = ValLoad[i];
							ValCurrent[i] = ValSet[i];
							SetRChannel(i, ValCurrent[i]);
							ack << "SET "<<i<<" "<< ValCurrent[i]<<"\n";
						} else {
							ack << "SET "<<i<<" "<< ValCurrent[i]<<"\n"; // CHECKME return ValSet ?
						}
						break;
					case CH_R0V:
					case CH_R1V:
						if(value != "") {
							ValVirtual[i-CH_R0V][channels&0xff] = ValLoad[i];
							Changed(true);
						}
						ack << "SET "<<(i-CH_R0V)<<"V"<<(channels&0xff)<<" ";
						ack << ValVirtual[i-CH_R0V][channels&0xff] <<"\n";
						break;
					case CH_DOUT:
						SetDOUT((unsigned long)(ValCurrent[i]=ValLoad[i]));
						ack << "SET DOUT "<<(unsigned long)(ValCurrent[i])<<"\n";
						break;
					default:
						ack << "ERR "<< stream->m_fd<<" SET "<<i<<" impossible\n";
						goto error_jump;
						break;
					}
				}
			}
		} else {
			// --------------------------------------------------LOAD
			for(int i=CH_R0; i<CH_N; i++) {
				if(channels & CH(i)) {
					switch(i) {
					case CH_R0:
					case CH_R1:
						ack << "LOAD "<<i<<" "<< ValLoad[i]<<"\n";
						break;
					case CH_DOUT:
						ack << "LOAD DOUT "<<(unsigned long)(ValLoad[i])<<"\n";
						break;
					default:
						ack << "ERR LOAD "<<i<<" impossible\n";
						break;
					}
				}
			}
			//printACK(ack.str()); //temp
		}
	} else if(cmd == "CLEAR") {
		channels = ParseChannel(chan);
		if(channels == CH_INVALID) {
			ack << "ERR "<< stream->m_fd<<" invalid channel CLEAR\n";
			ack << line << "\n";
			goto error_jump;
		}
		ret = can_clear(channels); // Changed(true) called inside
		if(ret) {
			ack << "ERR "<< stream->m_fd<<" error CLEAR\n";
			ack << line << "\n";
			goto error_jump;
		}
		ack << "OK "<< stream->m_fd<< "\nCLEAR " << chan << "\n";
	} else if(cmd == "GET") {
		channels = ParseChannel(chan);
		if(channels == CH_INVALID) {
			ack << "ERR "<< stream->m_fd<<" invalid channel GET\n";
			ack << line << "\n";
			goto error_jump;
		}
		ack << "OK "<< stream->m_fd<<"\n";
		for(int i=CH_R0; i<CH_N; i++) {
			if(channels & CH(i)) {
				switch(i) {
				case CH_R0:
				case CH_R1:
					ack << "SET "<<i<<" "<< ValCurrent[i]<<"\n";
					cout<<"val : "<<ValCurrent[i]<<"\n";
					break;
				case CH_R0V:
				case CH_R1V:
					ack << "SET "<<i-CH_R0V<<"V"<<(channels&0xff)<<" "<< ValVirtual[i-CH_R0V][channels&0xff]<<"\n";
					break;
				case CH_DOUT:
					ack << "SET DOUT "<<(unsigned long)(ValCurrent[i])<<"\n";
					break;
				case CH_DIN:
					ack << "GET DIN "<<((unsigned long)(ValCurrent[i]) & DIN_MASK )<<"\n";
					break;
				default:
					break;
				}
			}
		}
	} else if(cmd == "MODE") {
		if(chan == "LOCAL") {
			Mode[CH_R0] = Mode[CH_R1] = LOCAL;
			TransitionMODE(CH_R0,Mode[CH_R0]);
			TransitionMODE(CH_R1,Mode[CH_R1]);
		} else if(chan == "REMOTE") {
			Mode[CH_R0] = Mode[CH_R1] = REMOTE;
			TransitionMODE(CH_R0,Mode[CH_R0]);
			TransitionMODE(CH_R1,Mode[CH_R1]);			
		} else if(chan == "") {
		} else {
			ack << "ERR "<< stream->m_fd<<" invalid MODE :"<<chan<<":\n";
			ack << line << "\n";
			goto error_jump;
		}
		ack << "OK "<< stream->m_fd<<"\n";
		ack << "MODE ";
		ack << ((Mode[CH_R0]==LOCAL) ? "LOCAL":"REMOTE");
		ack << "\n";
		// CHANGE removed in V3.3 as not in spec FL01DE0000-00088-19/4 
		//ack << "SET 0 "<< ValCurrent[0]<<"\n";
		//ack << "SET 1 "<< ValCurrent[1]<<"\n";
	} else if(cmd == "IP") {
		string gateway;
		input >> gateway;

		if(chan.length()) {
			ValIpAddr = chan;
		} else {
			// no error, use "IP" without parameters to query things
//			ack << "ERR "<< stream->m_fd<<"\n";
//			ack << line << "\n";
//			goto error_jump;
		}
		if(value.length()) 
			ValIpNetmask = value;
		if(gateway.length())
			ValIpGateway = gateway;
		if( ValIpAddr == "0" ) {
			ValIpAddr = "0.0.0.0";
		}

		Changed(true);
		ack << "OK "<< stream->m_fd<<"\n";
		ack << "IP " << ValIpAddr << " " << ValIpNetmask << " " << ValIpGateway << "\n";
	} else if(cmd == "RST" || cmd == "QUIT" || cmd == "EXIT") {
		ack << "OK "<< stream->m_fd<<"\nRST\n";
		do_rst = true;
	} else if(cmd == "ECHO") {
		val = strtol(chan.c_str(), &end_ptr, 10);
		ack << "OK " << stream->m_fd << "\nECHO ";
		stream->m_flags = val;
		ack << stream->m_flags << "\n";
	} else if(cmd == "TYPE") {
		ack << "OK "<< stream->m_fd<<"\nTYPE ";
		if(ResiType & RESI_T_M) {
			ack << "RESI 3 2.6M\n";
			ack << "RMIN 0\nRMAX "<< RESI_R_MAX_M <<"\n";
			ack << "RSTEP 10\nROPEN -1\nINP_N 4\nOUT_N 4\nVIRT_N 256\n";
		} else {
			ack << "RESI 2 266K\n";
			ack << "RMIN 0\nRMAX "<< RESI_R_MAX <<"\n";
			ack << "RSTEP 1\nROPEN -1\nINP_N 4\nOUT_N 4\nVIRT_N 256\n";
		}
		ack << "SW "<< version << "\n";
		ack << "SN " << serialnr << "\n";
	} else if(cmd == "CFRAME"){
		string ident_length=chan;
		if(ident_length == "29"){
			ext=1;
			can_identifier=536870911;
			Changed(true);
		}
		else if(ident_length == "11"){
			ext=0;
			can_identifier=2047;
			Changed(true);
		}
		else if(ident_length != "") {
			ack << "ERR "<< stream->m_fd<<" wrong parameter "<<chan<<"\n";
			ack << line << "\n";
			goto error_jump;
		}
		ack << "OK "<< stream->m_fd<<"\n";
		ack << "CFRAME "<<(ext ? "29": "11")<<"\n";
		
	} else if(cmd == "CBAUDRATE"){
		int ret;
		if(chan != "") {
			int n = atoi(chan.c_str());
			if(n>0) {
				ret=set_bitrate(can_fd, n);
			} else {
				ret = -1;
			}
			if(ret != 0){
				ack << "ERR "<< stream->m_fd<<" wrong parameter "<<chan<<"\n";
				ack << line << "\n";
				goto error_jump;
			}
			can_bitrate = n;
			Changed(true);
		}
		ack << "OK "<< stream->m_fd<<"\n";
		ack << "CBAUDRATE "<<can_bitrate<<"\n";
		// Changed(true);
	} else if(cmd == "CRID"){
		if(chan != "") {
			int getRID = atoi(chan.c_str());
			if(getRID < 0 || getRID > can_identifier) {
				ack << "ERR "<< stream->m_fd<<" wrong parameter "<<chan<<"\n";
				ack << line << "\n";
				goto error_jump;
			}
			rid=getRID;
			Changed(true);
		}
		ack << "OK "<< stream->m_fd<<"\n";
		ack << "CRID "<<rid<<"\n";	
	} else if(cmd == "CBRID") {
		if(chan != "") {
			int getBRID = atoi(chan.c_str());
			if(getBRID < 0 || getBRID > can_identifier){
				ack << "ERR "<< stream->m_fd<<" wrong parameter "<<chan<<"\n";
				ack << line << "\n";
				goto error_jump;
			}
			brid = getBRID;
			Changed(true);
		}
		ack << "OK "<< stream->m_fd<<"\n";
		ack << "CBRID "<<brid<<"\n";
	} else if(cmd == "CTID") {
		if(chan != "") {		//chan here is ID
			int getTID = atoi(chan.c_str());
			if(getTID < 0 || getTID > can_identifier) {
				printf("can_identifier: %d\n", can_identifier);
				ack << "ERR "<< stream->m_fd<<" wrong parameter "<<value<<"\n";
				ack << line << "\n";
				goto error_jump;
			}
			tid=getTID;
			Changed(true);
		}
		ack << "OK "<< stream->m_fd<<"\n";
		ack << "CTID "<<tid<<"\n";
	} else if(cmd == "VERSION") {
		// ignore

	} else if(cmd == "CONFIG") {
		// TODO switch to quiet (no-ack) mode

	} else if(cmd == "END") {
		// TODO switch to normal ACK mode
		
	} else if(cmd == "SAVE") {
		WriteConfig();
		SaveConfig(0);
		
		ack << "OK "<< stream->m_fd<<"\n";
		ack << "SAVE\n";
	} else if(cmd[0] == '#') {
		// comment line, ignore
	} else if(cmd.length() == 0) {
		// empty line, ignore
	} else {
		ack << "ERR "<< stream->m_fd<<" unknown command\n";
		ack << line << "\n";
	}

error_jump:
	ack << "\r\n";
//	cerr << ack.str();
	WriteToAll(ack.str(), LineBuffer::F_ECHO_OTHER);
	// if F_ECHO_OTHER flag is not there, send ACK to this stream anyway
	if(!(stream->m_flags & LineBuffer::F_ECHO_OTHER)) {
		ret = write(stream->m_fd, ack.str().data(), ack.str().length());
	}
	if(do_rst) {
		DoRST();
		// this function will not exit, therefore it is done after ACK is sent
	}
	return 0;
}

#define IPMAX_L 40
char ipnum[IPMAX_L]="?", netmask[IPMAX_L]="?", tmp[IPMAX_L]="?", gateway[IPMAX_L]="?";

int ReadIPNum(void) {
	FILE *fin=NULL;
	int ret;
	system("/sbin/ifconfig eth0  | grep 'inet addr:' > /tmp/netstat.txt");
	system("/sbin/route -n  | grep '^0.0.0.0' >> /tmp/netstat.txt");
	fin = fopen("/tmp/netstat.txt", "r");
	if(fin==NULL) {
		ostringstream ack;
		ack << "ERR\n\r\n";
		WriteNet(fdUdp, ack.str().data(), ack.str().length());
		return -1;
	}
	ret = fscanf(fin, " inet addr:%39s Bcast:%39s Mask:%39s 0.0.0.0 %39s",
		ipnum, tmp, netmask, gateway);
	fprintf(stderr, "ReadIPNum ret %d ipnum %s tmp %s netmask %s gateway %s",
		ret, ipnum, tmp, netmask, gateway);
	return ret;
}

int HandleUdp(char * str) {
	int ret;
	string cmd(str);
	transform (cmd.begin(),cmd.end(), cmd.begin(), (int(*)(int)) toupper);
	istringstream input(cmd);
	ostringstream ack;

	input >> cmd; // get only the first word
	toaddr = fromaddr;
	if(recvaddr.sin_addr.s_addr == htonl(INADDR_BROADCAST))  {
		toaddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	}

	if(cmd == "RESIIP") {
		ret = ReadIPNum();
		if(ret>0) {
			ack << "OK\nIP " << ipnum << " " << netmask;
			if(ret>=4) {
				ack << " " << gateway;
			}
		} else {
			ack << "ERR";
		}
		ack << "\n\r\n";
	} else if(cmd == "IP") {

	} else if(cmd == "RST") {
	} else {
		// ignore not implemented
		// just for debug now
		ack << "ERR " << cmd << "\n\r\n";
	}
	WriteNet(fdUdp, ack.str().data(), ack.str().length());
	return 0;
}

void Watchdog(void) {
	UINT4 tmp = PortRead(CTRL1_A);
	tmp ^= CTRL1_WDI;
	PortWrite(CTRL1_A,tmp);
}

int MainResiLoop(void) {
 	printf("now we are in MainResiLoop()================== \n");

	char buf[500];
	struct timeval tvTimeoutTmp;
	int fdSelectMax;
	fd_set readsel;
	int ret;
	t_STREAM_LIST::iterator i;
	
	time0 = gettime();
	tsStarted = tvNow.tv_sec;
	tvSelectTimeout.tv_sec = 0;
	tvSelectTimeout.tv_usec = 10000; // wake up every 10 ms even without data
	for(int chan=0; chan<CH_R_N; chan++) {
		LastDIN[chan] = -1;
		ConfirmedDIN[chan] = -1;
		tLastDINChanged[chan] = time0;
	} 

	
	while(1) {

		// 
welche FDS müssen geprüft werden von 0 - fdSelectMax

		tvTimeoutTmp = tvSelectTimeout;
		fdSelectMax=-1;
		FD_ZERO(&readsel);
		t_STREAM_LIST::iterator i;
		
		if(fdNet>=0) {
			FD_SET(fdNet, &readsel);
			fdSelectMax = fdNet;
		}
		if(fdUdp>=0) {
			FD_SET(fdUdp, &readsel);
			if(fdSelectMax<fdUdp) {
				fdSelectMax = fdUdp;
			}
		}
		if(can_fd>=0)
			FD_SET(can_fd, &readsel);		/* watch on fd for CAN */
			if(fdSelectMax<can_fd) {
				fdSelectMax = can_fd;
			}


		for(i=streams.begin(); i!=streams.end(); ++i) {
			// here streams list must not be changed
			FD_SET((*i)->m_fd, &readsel);
			if((*i)->m_fd > fdSelectMax)
				fdSelectMax = (*i)->m_fd;
			}
		fdSelectMax++; // must be max_fd+1		
		select(fdSelectMax, &readsel, NULL, NULL, &tvTimeoutTmp);

----------------

		gettime();
		Watchdog();
		
		for(i=streams.begin(); i!=streams.end(); ) {
			if(FD_ISSET((*i)->m_fd, &readsel)) {
				ret = read((*i)->m_fd, buf, sizeof(buf)-1);
				if(ret>0) {
					buf[ret] = 0;
					(*i)->AddData(buf);
				} else if(ret == 0) {
					WARN("read returned 0 fd %d errno %d, closing", (*i)->m_fd, errno);
					close((*i)->m_fd);
					delete *i;
					i = streams.erase(i);
					// erase itself returns next element, no need to increment i
					continue; 
				} else if(ret < 0){
					WARN("read returned %d fd %d errno %d, closing", ret, (*i)->m_fd, errno);
					close((*i)->m_fd);
					delete *i;
					i = streams.erase(i);
					continue;
				}
			}
			++i;		
		}
		if(fdNet>=0 && FD_ISSET(fdNet,&readsel)) {
			// accept new connection
			struct sockaddr_in address;
			socklen_t addr_len = sizeof(address);
			ret = accept(fdNet, (struct sockaddr *) &address, &addr_len);
			if(ret >=0) {
				streams.push_front(new LineBuffer(ret, HandleCommand));
				printf("new connection from 0x%x:0x%x\n", address.sin_addr.s_addr, address.sin_port);
			} else {
				WARN("accept failed %d errno %d", ret, errno);
			}

		}
		if(fdUdp>=0 && FD_ISSET(fdUdp,&readsel)) {
			ret = ReadNet(fdUdp,buf, sizeof(buf)-1);
			if(ret>=0) {
				buf[ret] = 0;
				ret = HandleUdp(buf);
			}
		}
		//=================================================================CAN
		if( FD_ISSET(can_fd,&readsel) ){
			if(GetMessage()!=1)
				fprintf(stderr,"can read error\n");
			else {
				if ( (brid==rx.id) || (rid == rx.id) ){
					ret = CanHandleCommand();
					if(ret >= 0)
						SendACK(can_fd,ret);
				}
				else {
					if(debug>2)
						fprintf(stderr, "CAN command 0x%x is not for me 0x%x\n", rx.id, rid);
				}
			}
		}
		//=================================================================

		gettime();
		ReadDIN();
		StateMachineDIN();
		if(time_v_changed && ((tnow-time_v_changed)>1000000)) {
			fprintf(stderr, "autosaving config\n");
			WriteConfig();
			SaveConfig(0);
		}
	}
	return 0;
}

void WriteConfig(void) {
	int chan;
	int index;
	fstream fout;

	fout.open (config_name, fstream::out | fstream::trunc);

	fout << "CONFIG\n";
	fout << "\n";
	fout << "VERSION " << version << "\n";
	if(ValConfigVersion>=0)
		fout << "CFGSN " << ++ValConfigVersion << "\n";
	fout << "IP " << ValIpAddr << " " << ValIpNetmask << " " << ValIpGateway << "\n";
	fout << "CBAUDRATE "<< can_bitrate <<"\n";
	if(ext) 
		fout << "CFRAME 29\n";
	else
		fout << "CFRAME 11\n";
	fout << "CRID "<< rid <<"\n";
	fout << "CBRID "<< brid <<"\n";
	fout << "CTID "<< tid <<"\n";
	fout << "\n";

	for(chan=0; chan<CH_R_N; chan++) {
		for(index=0; index<INDEX_MAX; index++) {
			// TODO really read from a file
			if(ValVirtual[chan][index] >= 0) {
				fout << "SET "<<chan<<"V"<<index<<" "<<ValVirtual[chan][index]<<"\n";
			}
		}
	}
	// TODO save ValSet and Mode

	fout << "END\n";

	fout.close();
	Changed(false);
}

void ReadConfig(void) {
	int chan;
	int index;
	e_MODE mode = REMOTE;
	FILE *fin;
	LineBuffer buf(2, HandleCommand);

	for(chan=0; chan<CH_R_N; chan++) {
		for(index=0; index<INDEX_MAX; index++) {
			// TODO really read from a file
			ValVirtual[chan][index] = -1;
		}
	}

	fin = fopen(config_name, "r");
	if(fin) {
		char b[100];
		while(fgets(b, sizeof(b), fin)) {
			buf.AddData(b);
		}
		fclose(fin);
	}

	// now check whether to switch to LOCAL mode
	for(chan=0; chan<CH_R_N; chan++) {
		for(index=0; index<INDEX_MAX; index++) {
			if(ValVirtual[chan][index]>=0) {
				mode = LOCAL;
			}
		}
	}
	
	for(chan=0; chan<CH_R_N; chan++) {
		ValSet[chan] = -1;
		TransitionMODE(chan, mode); // Reset Index and set R
	}
	Changed(false); // prevent config from being written
}

void show_usage(void) {
	printf("resi-server options:\n"
	"-i	: intercative (read commands also from stdin)\n"
	"-o file.log : log to a file\n"
	"-c device_or_file : read commands/write responses to specified file\n"
	"-q	: quiet\n"
	"-h	: this help\n"
	);
}

/**
	handling command line options, calling functions, main program loop 
	@param argc, char * argv[]	- command line options
	@return  0					- OK
			-1					- ERROR */
int main(int argc, char * argv[]) {
	int ret;
	int opt;
	myport = DEFAULT_SERVER_PORT;
	myudpport = DEFAULT_SERVER_PORT;
	fdUdp = fdNet = fdIn = -1;

	fLog = NULL;
	signal(SIGPIPE, SIG_IGN); // ignore closed pipe signals
	
	// CHECKME what is "^" below ?
	while((opt=getopt(argc, argv, "io:w:d:p:u:l:c:r:bM:e^h"))>0) {
		switch(opt) {
		case 'i': // interactive or stdin
			fdIn = 0;
			break;
		case 'o': // log file
			fLog = fopen(optarg,"a");
			break;
		case 'w':
			watchdog = 1;
			sscanf(optarg, "%d", &watchdog);
			if(!watchdog) {
				PortWrite(CTRL1_A, PortRead(CTRL1_A) & ~CTRL1_WDI_EN);
			}
			break;
		case 'd':
			sscanf(optarg, "%d", &debug);
			break;
		case 'p':
			sscanf(optarg,"%hi", &myport);
			break;
		case 'r':
			ReadConfigFlash(optarg);
			break;
		case 'b':
			ReadConfigUboot();
			break;
		case 'e':
			if(watchdog) {
				DMSG("enabling watchdog and exiting");
				PortWrite(CTRL1_A, PortRead(CTRL1_A) | CTRL1_WDI_EN);
			}
			exit(0);
			break;
		case 'u':
			sscanf(optarg,"%hi", &myudpport);
			break;
		case 'l': // LCD contrast
			sscanf(optarg, "%i", &LcdElVol);
			break;
		case 'c':
			if ((ret = OpenSerial(optarg,B115200,0)) < 0) {
				WARN("could not open serial port :%s:", optarg);
			} else {
				DMSG("open serial port :%s: OK", optarg);
				streams.push_front(new LineBuffer(ret, HandleCommand));
			}
			break;
		case 'q':
			quiet ++;
			break;
		case 'h':
			show_usage();
			break;
		case 'M':
			sscanf(optarg, "%d", &ResiType);
			break;
		default:
			break;
		}
	}

	if(!quiet)
		fprintf(stderr, "%s (C) 2008 Wolf&Woelfel GmbH http://www.wolf-woelfel.de/\n", version);

	// it must have at least one connection while starting
	// if it is /dev/null it will be closed anyway on the first read
	
	if(fdIn>=0) {
		streams.push_front(new LineBuffer(fdIn, HandleCommand));
	}

	if(( can_fd = open("/dev/can0", O_RDWR )) < 0 ) {
		fprintf(stderr,"Error opening CAN device %s\n","/dev/can0");
		exit(1);
	}
	set_bitrate(can_fd, can_bitrate);

	fdNet = OpenListeningTcp(myport);
	fdUdp = OpenListeningUdp(myudpport);
	memset(&myaddr, 0, sizeof(myaddr));
	memset(&fromaddr, 0, sizeof(fromaddr));
	memset(&toaddr, 0, sizeof(toaddr));

	initParport(); // open GPIO, setup it and reset LCD
	initParportControl(); // stop reset
	LcdInit();
	LcdClear();
	/* display IP number for 2 s */
	ReadIPNum();
	if(ResiType & RESI_T_M) {
		DisplayBig(0,0,2, "RESI3 2.6M");
	} else {
		DisplayBig(0,0,2, "RESI2 266K");
	}
	Display57C(2,0,"IP ");
	Display57C(2,18,ipnum);
	Display57C(3,0,version);

	usleep(5000000);

	LcdClear();
	Display57C(0,0, "R1x:           <");
	Display57C(1,0, "R2x:           <");
	ReadConfig();

	/* open relays */
	string cmdstr("SET 0 -1");
	HandleCommand(*streams.begin(), cmdstr);
	HandleCommand(*streams.begin(), cmdstr = "SET 1 -1");
	HandleCommand(*streams.begin(), cmdstr = "SET DOUT 0");
	
	if(watchdog) {
		DMSG("enabling watchdog");
		PortWrite(CTRL1_A, PortRead(CTRL1_A) | CTRL1_WDI_EN);
	}
	MainResiLoop();

	return 0;       
}


/** @name GetNumber
	aliasing: deassign
	@param buf	- string buffer
	@param val	- alias value
	@return  0					- OK
			-1					- ERROR */
int GetNumber(char * buf, int * val) {
	if(!buf) {
		return -1;
	}
	if(sscanf(buf,"%i", val)==1) {
		return 0;
	}
	return -1;
}
int GetMessage()
{
	int got=0;
	unsigned long temp;
	got=read(can_fd, &rx, 1);
   	if( got > 0) {
		int j;
		if(rx.flags & MSG_ERR_MASK)
			WARN("CAN receive error: flags 0x%x MSG_ERR_MASK 0x%x\n", rx.flags, MSG_ERR_MASK);
		if(debug<2)
			return got;
		fprintf(stderr, "Received with ret=%d: %12lu.%06lu id=%ld\n",
			got, 
			rx.timestamp.tv_sec,
			rx.timestamp.tv_usec,
			rx.id);
		fprintf(stderr, "\tlen=%d msg=", rx.length);
		for(j = 0; j < rx.length; j++) {
			fprintf(stderr, " %02x", rx.data[j]);
		}
		fprintf(stderr, " flags=0x%02x\n", rx.flags );
		fflush(stderr);
	}
	else {
		WARN("CAN Received with ret=%d\n", got);
	}
	return got;
}
void SendACK(int fd,int retVal)
{
	//BYTE cmd=getCMD();
	int ret;
	cout<<"retVal"<<retVal<<"\n";
	memset(&tx, 0, sizeof(tx));
	if(ext){
		tx.flags = MSG_EXT; // if 29 bit	
	}
	//printf("tx.flags %d\n ", tx.flags);
	tx.id = tid;
	tx.data[0] = ACK_CAN;
	
	tx.data[1] = retVal;
	tx.length = 2;
	ret = write(fd, &tx, 1);
	if (ret <0)
		perror("can write error\n");
	if(tx.flags & MSG_ERR_MASK)
		printf("CAN receive error: MSG_ERR_MASK 0x%x\n", tx.flags);
}
//========================================================== CanHandleCommand()
/**
	process CAN input pqacket
	@return if < 0 reply packet was already sent, if >= 0 caller has to send OK_ERR packet with returned code
*/
int CanHandleCommand()
{
	BYTE cmd,chan;
	t_CHANNELS ch;
	
	cout<<"CanHandleCommand()\n";
	int ret=0;
	if(rx.length<1) {	//if true -> no command
		printf("ERR_CAN_LENGTH\n");
		return ERR_CAN_LENGTH;
	}
	cmd=0; chan=0;
	cmd = getCMD();
	chan=getChannel();
	
	printf("case chan = %d\n",chan);
	
	switch(chan)  {			//the same as ParseChannel()
	case 0: ch | CH_R0; break;
	case 1: ch | CH_R1; break;
	case 2: ch | CH_R0 | CH_R1; break;
	case 3: ch | CH_DOUT;  break;
	case 4: ch | CH_DIN; break;
	case 5: 
		if(rx.length<3){
			cout<<"ERR_CAN_LENGTH\n";
			return ERR_CAN_LENGTH;
		}
		if(rx.data[rx.length-2] != 0){
			cout<<"ERR_CAN_SZ\n";
			return ERR_CAN_SZ;
		}
		ch.setv( getuint(2, rx.data+rx.length-2) );
		ch | CH_R0V;
		break;
	case 6: 
		if(rx.length<3){
			cout<<"ERR_CAN_LENGTH\n";
			return ERR_CAN_LENGTH;
			}
		if(rx.data[rx.length-2] != 0){		
			cout<<"ERR_CAN_SZ\n";
			return ERR_CAN_SZ;
		}
		ch.setv(getuint(2,rx.data+rx.length-2)); 
		ch | CH_R1V; 
		break;
	case 7:	//only for set
		ch | CH_R0 | CH_R1 | CH_DOUT;
		break;
	default: 
		printf("wrong channel\n");
		return ERR_CAN_CHAN;
	}
	
	printf("cmd=%d\n",cmd);
	switch(cmd){
	case CAN_CMD_FIND:	
		ret = can_find();
		break;
	case CAN_CMD_LOAD:
		ret = can_load(ch);
		break;
	case CAN_CMD_SET:	
		ret = can_set(ch); 
		break;
	case CAN_CMD_GET:
		ret = can_get(ch);
		cout<<"case, ret get "<<ret<<"\n";
		break;	
	case CAN_CMD_MODE:			
		ret = can_mode();
		break;
	case CAN_CMD_CLEAR:
		ret = can_clear(ch);
		break;
	case CAN_CMD_SET_CAN_RID:
		ret = set_can_rid();
		break;
	case CAN_CMD_SET_CAN_TID:	
		ret = set_can_tid();
		break;
	case CAN_CMD_TYPE:
		ret = can_type();
		break;
	case CAN_CMD_RST:	
		can_rst();
		break;
	default: 
		printf("the command can not be recognized\n");
		ret = ERR_CAN_CMD;
		break;
	}
	return ret;
}

unsigned int getuint(int n,  const unsigned char *ptr)
{
	int i;
	int ret=0;
	for(i=0; i<n; i++) {
		ret <<=8;
		ret += ptr[i];
	}
	//printf("getuint()  %d = %d\n", n, ret);
	return ret;
}
//========================================================= can_load()
int can_load(t_CHANNELS ch)
{
	printf("can_load()\n");
	int ret = 0;
	int value=0;
	short int length=0;
	
	//DIN and CH_R0V and CH_R1V are not allowed
	if(ch&CH_DIN || ch &CH_R0V || ch &CH_R1V){	
		printf("ch 0x%x is not allowed : ERR_CAN_CHAN\n", (int)ch);
		return ERR_CAN_CHAN;
	}
	if((ch&CH_R0 || ch&CH_R1) && ch&CH_DOUT) {
		printf("ch 0x%x is not allowed : ERR_CAN_CHAN\n", (int)ch);
		return ERR_CAN_CHAN;
	}
	length = rx.length;
	printf("leng %d\n",rx.length);
	if(length <5){
		printf(" err, ERR_CAN_LENGTH\n");
		return ERR_CAN_LENGTH;
	}

	value = getuint(4, (rx.data+1));
	printf(" getuint() ok %d\n", value);
	
	for(int i=CH_R0; i<CH_N; i++) {
		if(ch&i) {
			switch(i) {
			case CH_R0:
			case CH_R1:
				if(value<-1 || value > RESI_R_MAX) {
					printf("ERR: value > RESI_R_MAX ");
					return ERR_CAN_VAL;
				}
				ValLoad[i] = value * RESOLUTION;
				cerr << "LOAD "<<i<<" "<< ValLoad[i]<<"\n";
				break;
			case CH_DOUT:
				if(value<0|| value > 15){
					printf("ERR: DOUT  > 15");
					return ERR_CAN_VAL;
				}
				ValLoad[i] = value;			
				cerr << "LOAD DOUT "<<(unsigned long)(ValLoad[i])<<"\n";
			break;
			default:
					cerr << "ERR LOAD "<<i<<" impossible\n";
			break;
			}
		}
	}
	printf("load() : ok, ret = %d\n", ret);
	return OK_CAN;
}
int can_set(t_CHANNELS ch){
	int value = -2;
	int ret = 0;
	unsigned int sz = ch.getv();
	short int length = rx.length;	//# of bytes

	//if chan == DIN -> err (not allowed)
	if( ch&CH_DIN){
		printf(": wrong channel\n");
		return ERR_CAN_CHAN;
	}
	//check for virt. channels
	if(ch&CH_R0V || ch & CH_R1V) {
		if(length !=7)  {
			printf("wrong XV length\n");
			return ERR_CAN_LENGTH;
		}
		if(sz>=INDEX_MAX) {
			printf("can_get:ERR_CAN_SZ");
			return ERR_CAN_SZ;
		}
	}
	
	if(length==1) {			//for set 7 or set 2
		// OK, LOAD has checked value!
	} else if(length<5){
		return ERR_CAN_LENGTH;
	} else {
		value = getuint(4,rx.data+1);
		if(length>5 && !(ch&CH_R0V || ch & CH_R1V)) {
			return ERR_CAN_LENGTH;
		}
		if(ch&CH_DOUT) {
			if(value<0  || value>RESI_DOUT_MAX) {
				return ERR_CAN_VAL;
			}
		}
		if(ch&CH_R0 || ch & CH_R1 || ch & CH_R0V || ch & CH_R1V) {
			if(value<-1  || value>RESI_R_MAX) {
				return ERR_CAN_VAL;
			}
		}
	}
	
	
	for(int i=CH_R0; i<CH_N; i++) {
		if(ch&i) {
			//ack << "cur "<< ValCurrent[i] << " load "<< ValLoad[i]<< " ";
			//ack << "val "<< value << " ";
			switch(i) {
			case CH_R0:
			case CH_R1:
				if(length>=5) {
					ValLoad[i] = value;
				}
				ValSet[i] = ValLoad[i];
				if(Mode[i]!=LOCAL) {
					ValCurrent[i] = ValSet[i];
					SetRChannel(i, ValCurrent[i]);
					//ack << "SET "<<i<<" "<< ValCurrent[i]<<"\n";
				} else {
					//ack << "SET "<<i<<" "<< ValCurrent[i]<<"\n"; // CHECKME return ValSet ?
				}
				break;
			case CH_R0V:
			case CH_R1V:
				if(length>=5) {
					ValLoad[i] = value;
					ValVirtual[i-CH_R0V][sz] = ValLoad[i];
					Changed(true);
				} // there is no else, because set XV without value is not allowed, length == 7 here
				//ack << "SET "<<(i-CH_R0V)<<"V"<<(channels&0xff)<<" ";
				//ack << ValVirtual[i-CH_R0V][channels&0xff] <<"\n";
				break;
			case CH_DOUT:
				if(length>=5) {
					ValLoad[i] = value;
				}				
				SetDOUT((unsigned long)(ValCurrent[i]=ValLoad[i]));
				//ack << "SET DOUT "<<(unsigned long)(ValCurrent[i])<<"\n";
				break;
			default:
				//ack << "ERR SET "<<i<<" impossible\n";
				printf("ERR SET can_");
				break;
			}
		}
	}
	return ret;
}
int can_find(){
	char buf[3];
	int sn = serialnr;
	int ret=0;
//check whether "RESI" and "WuW" are in can-frame
	if((rx.data[1]==0x52 && rx.data[2]==0x45 && rx.data[3]==0x53 && rx.data[4]==0x49) 
		&&(rx.data[5]==0x57 && rx.data[6]==0x75 && rx.data[7]==0x57) ){
		
		tx.id = tid;
		memset(tx.data, 0, 8);
		tx.data[7] = sn & 0xff;
		tx.data[6] = (sn>>8) & 0xff;
		tx.data[5] = (sn>>16) & 0xff;

		for( int i=1; i<5; i++){		// write/copy 'R'E'S'I'
			tx.data[i] = rx.data[i];
		}
	
		tx.length = 8;
		ret = write(can_fd, &tx, 1);
		if (ret == -1) {
			perror("can write error\n");
			return -3;
		}
		if(tx.flags & MSG_ERR_MASK)
			printf("CAN receive error: MSG_ERR_MASK\n");
		return -1;
	}
	else
		return -2;
}
int can_get(t_CHANNELS ch){
	printf("can_get()\n");

	unsigned int sz = ch.getv();
	cout<<"can_get(), channels = "<<(int)ch<<"\n";
	
	int value = 0;
	int done=0;
	int ret = 0;
	memset(tx.data, 0, 8);

	if(rx.length<1){
		cout<<"ERR_CAN_LENGTH\n";
		return ERR_CAN_LENGTH;
	}
	if(sz>=INDEX_MAX) {
		printf("can_get:ERR_CAN_SZ");
		return ERR_CAN_SZ;
	}
	
	for(int i=CH_R0; i<CH_N; i++) {
		if(ch & i) {
			cout<<"true for i :"<<i<<"\n";
			tx.length = 5;
			switch(i) {
			case CH_R0:
			case CH_R1:
				//ack << "SET "<<i<<" "<< ValCurrent[i]<<"\n";
				value = (int)(ValCurrent[i] / RESOLUTION);
				printf("val: %d\n", value);
				cout<<"val : "<<ValCurrent[i]<<"\n";
				tx.data[0] = i<<5;
				break;
			case CH_DOUT:
			case CH_DIN:
				//ack << "SET DOUT "<<(unsigned long)(ValCurrent[i])<<"\n";
				cout<<"channels "<<ch<<"\n";
				cout<<"DIN/DOUT - Value "<<(unsigned long)(ValCurrent[i])<<"\n";
				value = (unsigned long)(ValCurrent[i]); 
				tx.data[0] = (i+1)<<5;	//+1 because there are diffrec. between CAN channels defin. and  e_CHANNELS
				break;	
			case CH_R0V:
			case CH_R1V:
				cout<<"channels "<<sz<<"\n";
				cout<<"sz "<<sz<<"\n";
				value = (int) ((ValVirtual[i-CH_R0V][sz]) / RESOLUTION);
				printf("val: %d\n", value);
				tx.data[0] = (i+1)<<5;
				tx.data[5] = sz>>8;
				tx.data[6] =(sz&0xff);
				tx.length = 7;
				break;
			default:
				return ERR_CAN_CHAN;
				break;
			}
			assert(value>=-1);
			assert(value<=RESI_R_MAX);
			tx.data[1] = value>>24;
			tx.data[2] = value>>16;
			tx.data[3] = value>>8;
			tx.data[4] = value;					
			done++;
		}
	}
	if(done>1) {
		printf("done > 1\n");
		return ERR_CAN_CHAN;
	}
	tx.data[0] |= CAN_BIT_ACK | CAN_CMD_GET;
	ret = write(can_fd, &tx, 1);
	if (ret == -1) {
		perror("can write error\n");
		return ERR_CAN_CMD;
	}
	if(tx.flags & MSG_ERR_MASK)
		printf("CAN receive error: MSG_ERR_MASK\n");
	return -1;	//if ok return -1 else error-code
}
int can_mode() {
	
	BYTE b1=rx.data[1];
	short int length = rx.length;
	int ret = 0;
	if( length == 1){	//not allowed
		cout<<"Command without mode -> err\n";
		return ERR_CAN_LENGTH;
	}
	if (length == 2 ){ 
		if(b1 == 0) {//if local
			Mode[CH_R0] = Mode[CH_R1] = LOCAL;
			TransitionMODE(CH_R0,Mode[CH_R0]);
			TransitionMODE(CH_R1,Mode[CH_R1]);
		} 
		else if(b1 == 1) {//if remote
			Mode[CH_R0] = Mode[CH_R1] = REMOTE;
			TransitionMODE(CH_R0,Mode[CH_R0]);
			TransitionMODE(CH_R1,Mode[CH_R1]);			
		}
		if ( (b1!= 0) && (b1 != 1)){ 
			cout<<"Command with wrong mode -> err\n";
			return ERR_CAN_MODE;
		}
	}
	return ret;
}
int set_can_rid(){

	int ret = 0;
	unsigned int rx_rid = 0;
	unsigned int rx_sn = 0;
	short int length = 0;
	length = rx.length;
	rx_rid = getuint(4, (rx.data+1));
	rx_sn = getuint(3, (rx.data+5));
	if( length == 1){
		printf("set_can_rid without parameters not allowed\n");
		return ERR_CAN_LENGTH;
	}
	else if ( length ==  4 ){
		printf("set_can_rid without RID not allowed\n");
		return ERR_CAN_ID;
	}
	else if (serialnr == rx_sn){
		if( rx_rid >= 0 && rx_rid <= can_identifier){
			rid = rx_rid;
			cout<<"new rid is: "<<rid<<"\n";
			Changed(true);
			return ret;
		}
		else{
			printf("set_can_rid with wrong RID\n");
			return ERR_CAN_ID;
		}
	}
	cout<<"rid is: "<<rid<<"\n";
	return -1;	//SN not correct -> no answer
}
	
int set_can_tid(){
	
	int ret = 0;
	unsigned long rx_tid = 0;
	unsigned int rx_sn = 0;
	short int length = 0;
	length = rx.length;
	rx_tid = getuint(4, (rx.data+1));
	rx_sn = getuint(3, (rx.data+5));
	if( length == 1){
		printf("set_can_rid without parameters not allowed\n");
		return ERR_CAN_LENGTH;
	}
	else if ( length ==  4 ){
		printf("set_can_tid without TID not allowed\n");
		return ERR_CAN_ID;
	}
	else if (serialnr == rx_sn){
		if( rx_tid >= 0 && rx_tid <= can_identifier){
			tid = rx_tid;
			cout<<"new tid is: "<<tid<<"\n";
			Changed(true);
			return ret;
		}
		else{
			printf("set_can_tid with wrong TID\n");
			return ERR_CAN_ID;
		}
	}
	cout<<"tid is: "<<tid<<"\n";
	return -1;	//SN not correct -> no answer
}
int can_rst(){
	bool do_rst = false;
	do_rst = true;
	if(do_rst) {
		DoRST();
		//todo: how does it work, doRST?
		// this function will not exit, therefore it is done after ACK is sent
	}
}
int can_type(){
	int ret=0;
	tx.id = tid;
	memset(tx.data, 0, 8);
	int max=0;
	max = RESI_R_MAX;	
	tx.data[0] = ACK_TYPE;				//reply from can_type( )
	tx.data[1] = TYP;					//RESI-TYP
	tx.data[2] = conversion(RESOLUTION);
	tx.data[3] = max>>16;				//max R
	tx.data[4] = max>>8;
	tx.data[5] = max;
	tx.data[6] = version[6] - 48;
	tx.data[7] = version[8] - 48;
	tx.length = 8;
	ret = write(can_fd, &tx, 1);
	if (ret == -1) 
		perror("can write error\n");
	return -1;
}
int can_clear(t_CHANNELS ch){
	int ret=-1;
	if(debug>2)
		cout<<"clear("<<ch<<")\n";
	if((ch&CH_R0V) || (ch&CH_R0)) {
		for(int i=0; i<INDEX_MAX; i++) {
			if(ValVirtual[0][i] != -1)
				Changed(true);
			ValVirtual[0][i] = -1;
		}
		ret = 0;
	}
	if((ch&CH_R1V) || (ch&CH_R1)) {
		for(int i=0; i<INDEX_MAX; i++) {
			if(ValVirtual[1][i] != -1)
				Changed(true);
			ValVirtual[1][i] = -1;
		}
		ret = 0;
	}
	if(ret) {
		WARN("can_clear(), channel 0x%x is incorrect", ch);
		return ERR_CAN_CHAN;
	}
	return ret;
}
//========================================================= set_bitrate() 
int set_bitrate(int fd, int baud)
{
	int ret;
	fprintf(stderr, "set_bitrate %d\n", baud);
	Config_par_t  cfg;
	volatile Command_par_t cmd;

	cmd.cmd = CMD_STOP;
	ret = ioctl(fd, CAN_IOCTL_COMMAND, &cmd);

	cfg.target = CONF_TIMING;
	cfg.val1   = baud;
	ret = ioctl(fd, CAN_IOCTL_CONFIG, &cfg);
	if(ret == 0) {
		can_bitrate=baud;
		Changed(true);
	}	

	cmd.cmd = CMD_START;
	ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
	
	return 0;
}
BYTE getChannel(){
	BYTE chan;
	BYTE all = rx.data[0];
	chan = all & 0xe0;		//um den channel herauszuholen
	chan = chan>>5;
	return chan;
}
BYTE getCMD(){
	BYTE com;
	BYTE all = rx.data[0];
	com = all & 0x0f;		//um den Befehl herauszuholen
	return com;
}
/**	convert RESOLUTION according to specifikation.
	works only for y between 1 and 9 (resolution values like 12 or 0.15 not supported)
	@param reso RESOLUTION
	@return ret as integer value
*/
int conversion (double reso){
	
	string str_reso;
	int x,y;
	string conv;
	string str_y;

	std::stringstream ss_reso;
	int ret;
	
	ss_reso << reso;
	str_reso=ss_reso.str();
	
	int len = str_reso.length();
	string::size_type loc = str_reso.find( ".", 0 );

	if(loc != string::npos){	//with comma	
		str_y = str_reso.at(len-1);
		y=atoi(str_y.c_str());
		x = 10-len;
	}
	else{						//without comma
		str_y = str_reso.at(0);	
		y=atoi(str_y.c_str());
		x = 7+len;
	}
	assert(x>=0);
	assert(x<16);
	assert(y>=0);
	assert(y<16);
	ret= (x<<4) | y;
	return ret;
}

int openConfig(){

}
