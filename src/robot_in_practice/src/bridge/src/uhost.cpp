/*
 #***************************************************************************
 #*   Copyright (C) 2017-2023 by DTU
 #*   jca@elektro.dtu.dk
 #*
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */

#define REV_ID "$Id: uhost.cpp 239 2023-03-18 08:57:59Z jcan $"

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>
#include <netpacket/packet.h>
#include <unistd.h>
#include <math.h>

#include "uhost.h"
#include "uhandler.h"
#include "ubridge.h"
#include "ujoy.h"
#include <linux/wireless.h>
#include <sys/ioctl.h>


UHostIp hostip;

const char * getHostRev()
{
  const char * p1 = strstr(REV_ID, ".cpp");
  if (p1 != nullptr)
  {
    p1 += 5;
  }
  else
    p1 = REV_ID;
  return p1;
}

void UHostIp::setup()
{
  setSourceID("host");
  displayIP = true;
  lastIpCnt = 0;
  ledIndex = -1;
  gethostname(hostname, MHL);
  char s1[] = "ip name Host v4 IP";
  handler.handleCommand(this, s1, true);
  char s[] = "temp name CPU temperature.";
  handler.handleCommand(this, s, true);
  // start regular update
  start();
}


bool UHostIp::check_wireless(const char* ifname, char* protocol) 
{
  int sock = -1;
  struct iwreq pwrq;
  memset(&pwrq, 0, sizeof(pwrq));
  strncpy(pwrq.ifr_name, ifname, IFNAMSIZ);
  bool isWiFi = false;
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
  {
    perror("UHostIp::check_wireless: socket");
  }
  else if (ioctl(sock, SIOCGIWNAME, &pwrq) != -1) 
  {
    if (protocol) 
      strncpy(protocol, pwrq.u.name, IFNAMSIZ);
    isWiFi = true;
  }
  if (sock >= 0)
    close(sock);
  return isWiFi;
}

bool UHostIp::findWifiMACs()
{
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  bool changed = false;
  //  
  getifaddrs(&ifAddrStruct);
  ipsCnt = 0;
  macCnt = 0;
  char temp[MHL2];
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) 
  { // all interface names and types
    if (ifa->ifa_addr->sa_family == AF_PACKET ) 
    { // is a valid interface
      if (strcmp(ifa->ifa_name, "lo") != 0)
      { // not loop back - loop back is skipped
        // get MAC for this interface
        struct sockaddr_ll *s = (struct sockaddr_ll*)ifa->ifa_addr;
        char protocol[IFNAMSIZ] = {'\0'};
        if (s->sll_halen > 0 and check_wireless(ifa->ifa_name, protocol))
        { // it has a MAC and is a wifi connection
          snprintf(temp, MHL, "%s %02x:%02x:%02x:%02x:%02x:%02x (%s)", ifa->ifa_name,
                    s->sll_addr[0], s->sll_addr[1], s->sll_addr[2],
                    s->sll_addr[3], s->sll_addr[4], s->sll_addr[5], protocol);
          if (strcmp(temp, macs[macCnt]) != 0)
          {
            changed = true;
            strncpy(macs[macCnt], temp, MHL2);
//             printf("#### found MAC %d: %s %s\n", macCnt, macs[macCnt], protocol);
          }
          if (macCnt < MIPS - 1)
            macCnt++;
        }
      }
      //
    }
//     if (macCnt == 0)
//       printf("# UHostIp::findWifiMACs: Found no wifi MACs\n");
  }
  return changed;
}

bool UHostIp::findIPs()
{
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;
  bool changed = false;
  //  
  getifaddrs(&ifAddrStruct);
  ipsCnt = 0;
  macCnt = 0;
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) 
  { // all interface names and types
    if (ifa->ifa_addr->sa_family == AF_INET) 
    { // is a valid IP4 Address
      tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
      if (strcmp(ifa->ifa_name, "lo") != 0)
      { // not loop back - loopback is skipped
        int sl = strlen(addressBuffer);
        if (sl > 0 and strncmp(ips[ipsCnt], addressBuffer, sl) != 0)
        {
          changed = true;
          snprintf(ips[ipsCnt], MHL, "%s", addressBuffer); 
          printf("#### found IP %d: %s %s\n", ipsCnt, ifa->ifa_name, ips[ipsCnt]);
        }
        if (ipsCnt < MIPS - 1)
          ipsCnt++;
      }
    }
  }
  return changed;
}


void UHostIp::run()
{
  int loop = 0;
  const int MSL = 100;
  char s[MSL];
  while (isRunning())
  {
    findIPs();
    // update message
    updateIPlist();
    // also MAC
    bool changed = findWifiMACs();
    if (changed and macCnt > 0)
    { // make mac list message
      snprintf(maclist, MHL2, "mac %s\n", macs[0]);
      handler.handleCommand(this, maclist, true);  
      if (macCnt > 1)
        printf("# found more than 1 wifi interface! (ignored)\n");
    }
    // tell Regbot display about the host IP
    USource * regbot = bridge.findSource("regbot");
    if (regbot != nullptr)
    { // send display message with IP addresses to REGBOT
      snprintf(s, MSL, "regbot disp%s\n", ip4list);
      handler.handleCommand(this, s, true);
    }
    // measure CPU temperature
    snprintf(s, MSL, "temp %.2f\n", measureCPUtemp());
    handler.handleCommand(this, s, true);
    if (loop % 10 == 0)
    { // and software revision number
      snprintf(s, MSL, "rev %s\n", getHostRev());
      // remove last $ sign
      char * p1 = strrchr(s,'$');
      if (p1 != nullptr)
        *p1 = '\0';
      // make available
      handler.handleCommand(this, s, true);
    }
    //
    loop++;
    if (bridge.loop == loopBridge)
      printf("# UHost:run: bridge loop has stopped (%d == %d)\n", bridge.loop, loopBridge);
//     else
//       printf("# UHost:run: bridge loop is running´ (%d == %d)\n", bridge.loop, loopBridge);
    loopBridge = bridge.loop;
    if (joy.loop == loopJoy)
      printf("# UHost:run: Joy    loop has stopped (%d == %d)\n", joy.loop, loopJoy);
//     else
//       printf("# UHost:run: Joy    loop is running (%d == %d)\n", joy.loop, loopJoy);
    loopJoy = joy.loop;
    // repeat every 2 seconds
    sleep(1);
  }    
}


void UHostIp::updateIPlist()
{ // update IP list
  int n = 0;
  char * p1 = ip4list;
  for (int i = 0; i < ipsCnt; i++)
  {
    snprintf(p1, MHL2 - n, " %s", ips[i]);
    n += strlen(p1);
    p1 = &ip4list[n];
  }
  // see if it has changed
  if (strcmp(ip4list, ip4listLast) != 0)
  { // make IP list message
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "ip%s\n", ip4list );
    handler.handleCommand(this, s, true);
  }
}

float UHostIp::measureCPUtemp()
{
  FILE * temp;
  const char * tf = "/sys/class/thermal/thermal_zone0/temp";
  temp = fopen(tf, "r");
  float t = 0;
  if (temp != NULL)
  {
    const int MSL = 20;
    char s[MSL];
    char * p1 = s;
    int n = fread(p1, 1, 1, temp);
    int m = n;
    while (n > 0)
    {
      n = fread(++p1, 1, 1, temp);
      m += n;
    }
    s[m] = '\0';
    if (m > 3)
    {
      t = strtof(s, &p1);
    }
    //     printf("#got %d bytes (%g deg) as:%s\n", m, t/1000.0, s); 
    fclose(temp);
  }
  return t/1000.0;
}
