EMANE LTE Model
========

The EMANE LTE Model project contains EMANE radio model implementations
for the LTE ENB and UE waveforms. The models are used in conjunction with
the [srsLTE-emane](https://github.com/adjacentlink/srsLTE-emane)
project applications to emulate LTE networks within EMANE.

The EMANE UE and ENB LTE models are not implemented as individual
EMANE plugins, departing from from all prior EMANE models. Both
are contained in the libemanelte.so library. `srsenb-emane`
and `srsue-emane` use a feature, introduced in EMANE
1.2.3, to embed an EMANE emulator instance internally; `emane` does
not run as a separate application. The
[emane-embedded-example](https://github.com/adjacentlink/emane-embedded-example)
project provides a small example, for those interested.

The lastest stable version: 1.0.6.


---
## Build Instructions

1. Install the latest [pre-built EMANE bundle](https://github.com/adjacentlink/emane/wiki/Install). EMANE version 1.2.3 or later is **required**.

2. Build and install `emane-model-lte`.
   * [Centos 7](#centos-7)
   * [Fedora 33](#fedora-33)
   * [Ubuntu 18.04 and 20.04](#ubuntu-1804-and-2004)

3. Build and install [srsLTE-emane](https://github.com/adjacentlink/srsLTE-emane.git).


### Centos 7

Centos 7 requires an additional step of installing devtoolset-9 for
c++17 support.

```
sudo yum install autoconf automake git libtool libxml2-devel libpcap-devel pcre-devel libuuid-devel python-devel python-setuptools rpm-build make gcc-c++

sudo yum install centos-release-scl
sudo yum install devtoolset-9

git clone https://github.com/adjacentlink/emane-model-lte.git
cd emane-model-lte
scl enable devtoolset-9 "./autogen.sh && ./configure && make rpm"
sudo yum install .rpmbuild/RPMS/x86_64/*rpm
```

### Fedora 33
```
sudo dnf install autoconf automake git libtool libxml2-devel libpcap-devel pcre-devel libuuid-devel python-devel python-setuptools rpm-build make gcc-c++
git clone https://github.com/adjacentlink/emane-model-lte.git
cd emane-model-lte
./autogen.sh
./configure
make rpm
sudo dnf install .rpmbuild/RPMS/x86_64/*rpm
```

### Ubuntu 18.04 and 20.04

```
sudo apt-get install gcc g++ autoconf automake libtool libxml2-dev libprotobuf-dev python-protobuf libpcap-dev libpcre3-dev uuid-dev debhelper pkg-config python-setuptools protobuf-compiler git dh-python
git clone https://github.com/adjacentlink/emane-model-lte.git
cd emane-model-lte
./autogen.sh
./configure
make deb
sudo dpkg -i .debbuild/emane-model-lte*.deb
```

---
## Demonstration

The EMANE LTE demonstration is written as
an [letce2](https://github.com/adjacentlink/letce2) project.  If you
installed the EMANE bundle, you've already installed the tools
to run the demos.

The demonstration is contained in the `emane-model-lte/demo`
subdirectory. It is also installed to `/usr/share/emane-model-lte`
if you installed from a binary package instead of working with the
github source repository.


### Demonstration

The demonstration, `emane-model-lte/demo/two_ues`, launches a
an LTE network with two UEs and 1 ENB. The srsLTE-emane applications run inside four
[LXC Containers](https://linuxcontainers.org/), one for the LTE EPC, one
for the ENB and one for each UE. The containers also run `sshd` and
are reachable at the backchannel addresses over the `letce0` Linux
bridge created by the demo:


```
# lxc container backchannel addresses
10.88.1.101 lxc-epc-01
10.88.1.91  lxc-enb-01
10.88.1.1   lxc-ue-01
10.88.1.2   lxc-ue-02

# radio network addresses
172.16.0.101 epc-01
172.16.0.1 ue-01
172.16.0.2 ue-02
```

The `srsepc-emane` and `srsue-emane` applications create Linux tuntap
devices (the `172.16.0.x` addreses) as IP entry points into the
LTE network. The terminal snippets that follow use the hostnames (above)
in the prompt to show where the command is executed. `host` means
the computer host computer where you run the demo.

The ENB and UE radio models running inside the emane instance embedded
in `srsenb-emane` and `srsue-emane` take a single XML configuration file
(default name: `emanelte.xml`) instead of the usual hierarchy of XML
files when running `emane` standalone. Here is `two_ues/ue-01/emanelte.xml`:

```XML
<?xml version="1.0" encoding="UTF-8"?>
<emanelte>
  <platform id="1"
            loglevel="2"
            logfile="/home/me/working/emane-model-lte/demo/two_ues/persist/ue-01/var/log/emane.log"
            otamanagerdevice="backchan0"
            eventservicedevice="backchan0">

    <radiomodel pcrcurveuri="pcr.xml"
                resourceblocktxpower="0.0"/>

    <phy noisemode="all"
         propagationmodel="precomputed"
         subid="12"/>

  </platform>
</emanelte>
```

Use `letce2` to build and start the demo:

```
[me@host working]$ cd emane-model-lte/demo/two_ues
[me@host two_ues]$ letce2 lxc build bootstrap.cfg
[me@host two_ues]$ letce2 lxc start
host: waiting for letce0
host: letce0 found
Cannot change rx-checksumming
Actual changes:
tx-checksumming: off
                 tx-checksum-ip-generic: off
                 tcp-segmentation-offload: off

   ...
running host start.local
setting initial pathloss to 200 for 1:30 on device letce0 at 11:23:10
finished pathloss on device letce0 at 11:23:10
```

Inspect the network devices on one of the UE nodes. `srsue-emane` creates
the `srsue` device once attached to the EPC:

```
[me@host two_ues]$ ssh lxc-ue-01
[me@ue-01 ~]$ ifconfig
backchan0 Link encap:Ethernet  HWaddr e2:b3:86:0a:09:a6
          inet addr:10.88.1.1  Bcast:10.88.1.255  Mask:255.255.255.0
          inet6 addr: fe80::e0b3:86ff:fe0a:9a6/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:111908 errors:0 dropped:0 overruns:0 frame:0
          TX packets:37041 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:47777410 (47.7 MB)  TX bytes:7940042 (7.9 MB)

lo        Link encap:Local Loopback
          inet addr:127.0.0.1  Mask:255.0.0.0
          inet6 addr: ::1/128 Scope:Host
          UP LOOPBACK RUNNING  MTU:65536  Metric:1
          RX packets:13 errors:0 dropped:0 overruns:0 frame:0
          TX packets:13 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1
          RX bytes:2333 (2.3 KB)  TX bytes:2333 (2.3 KB)

srsue     Link encap:UNSPEC  HWaddr 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00
          inet addr:172.16.0.1  P-t-P:172.16.0.1  Mask:255.255.255.0
          UP POINTOPOINT RUNNING NOARP MULTICAST  MTU:1500  Metric:1
          RX packets:0 errors:0 dropped:0 overruns:0 frame:0
          TX packets:0 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:500
          RX bytes:0 (0.0 B)  TX bytes:0 (0.0 B)
```

Ping the EPC:

```
[me@ue-01 ~]$ ping epc-01
PING epc-01 (172.16.0.101) 56(84) bytes of data.
64 bytes from epc-01 (172.16.0.101): icmp_seq=1 ttl=64 time=457 ms
64 bytes from epc-01 (172.16.0.101): icmp_seq=2 ttl=64 time=37.2 ms
64 bytes from epc-01 (172.16.0.101): icmp_seq=3 ttl=64 time=36.1 ms
64 bytes from epc-01 (172.16.0.101): icmp_seq=4 ttl=64 time=34.8 ms
     ...
```

All of the srsLTE-emane applications expose shared code statistics at
an [OpenStatistic](https://github.com/adjacentlink/openstatistic)
endpoint. The endpoint default port is 47100. Use `ostatistic` to
inspect values:


```
[me@host two_ues]$ ostatistic -p 47100 lxc-epc-01 get table
BearerTable
| Dst | eNBTEID | eNBAddr | IMSI | EBI | Time |

DownlinkDropTrafficTable
| Src | Dst | Count | Bytes | Time |

DownlinkTrafficTable
| Src        | Dst        | Count | Bytes | Time       |
| 172.16.0.1 | 172.16.0.2 | 13    | 1372  | 1548764479 |
| 172.16.0.1 | 172.16.0.3 | 12    | 1344  | 1548764479 |
| 172.16.0.2 | 172.16.0.3 | 12    | 1008  | 1548764479 |
| 172.16.0.3 | 172.16.0.2 | 12    | 1008  | 1548764479 |

UplinkTrafficTable
| Src        | Dst        | Count | Bytes | Time       |
| 172.16.0.2 | 172.16.0.1 | 3     | 252   | 1548764466 |
| 172.16.0.2 | 172.16.0.3 | 12    | 1008  | 1548764479 |
| 172.16.0.3 | 172.16.0.2 | 12    | 1008  | 1548764479 |
```

The embedded EMANE instances running in `srsenb-emane` and
`srsue-emane` expose radio model statistics via the EMANE Control
Port. Access these with `emanesh` using the default port, 47000:

```
[me@host two_ues]$ emanesh lxc-ue-01 get table nems mac UplinkTxPUSCHFrequencyCounts
nem 1   mac UplinkTxPUSCHFrequencyCounts
| Frequency  | 0.1 | 0.2 | 1.1 | 1.2 | 2.1 | 2.2 | 3.1 | 3.2 | 4.1 | 4.2 | 5.1 | 5.2 | 6.1 | 6.2 | 7.1 | 7.2 | 8.1 | 8.2 | 9.1   | 9.2   |
| 2566259936 | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0     | 0     |
| 2566079936 | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0     | 0     |
| 2565899936 | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 53047 | 53047 |
| 2565719936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 0   | 0   | 0   | 0   | 3   | 3   | 0   | 0   | 0   | 0   | 8     | 8     |
| 2565539936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 0   | 0   | 0   | 0   | 3   | 3   | 0   | 0   | 0   | 0   | 8     | 8     |
| 2565359936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 0   | 0   | 0   | 0   | 3   | 3   | 0   | 0   | 1   | 1   | 8     | 8     |
| 2565179936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 0   | 0   | 0   | 0   | 3   | 3   | 0   | 0   | 1   | 1   | 8     | 8     |
| 2564999936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 0   | 0   | 0   | 0   | 3   | 3   | 0   | 0   | 1   | 1   | 8     | 8     |
| 2564819936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 0   | 0   | 0   | 0   | 3   | 3   | 0   | 0   | 1   | 1   | 8     | 8     |
| 2564639936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 1   | 1   | 0   | 0   | 3   | 3   | 1   | 1   | 15  | 15  | 8     | 8     |
| 2564459936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 1   | 1   | 1   | 1   | 3   | 3   | 3   | 3   | 15  | 15  | 8     | 8     |
| 2564279936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 1   | 1   | 1   | 1   | 3   | 3   | 3   | 3   | 20  | 20  | 8     | 8     |
| 2564099936 | 0   | 0   | 0   | 0   | 0   | 0   | 2   | 2   | 1   | 1   | 1   | 1   | 3   | 3   | 3   | 3   | 20  | 20  | 8     | 8     |
| 2563919936 | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0     | 0     |
| 2563739936 | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0     | 0     |
```

[OpenTestPoint](https://github.com/adjacentlink/opentestpoint-probe-lte.git)
enables access to both sets of statistics via one uniform data
stream. [OpenTestpoint LTE probes](https://github.com/adjacentlink/opentestpoint-probe-lte.git)
defines the probes specific to LTE. Follow the build and install directions
there if building from source.

The OpenTestPoint Daemon, `otestpointd` runs in each LXC
container. It takes an XML configuration file, `otestpointd.xml`, that
lists the probes to load. Here is the file for `enb-01`:

```XML
<otestpoint id="enb-01" discovery="0.0.0.0:8881" publish="0.0.0.0:8882">
  <probe configuration="probe-srslte-enb.xml">
    <python module="otestpoint.lte.srsenb" class="SRSENB"/>
  </probe>
  <probe configuration="probe-emanelte-enb.xml">
    <python module="otestpoint.lte.emaneenb" class="EMANEENB"/>
  </probe>
</otestpoint>
```

In this case, `otestpointd` loads probes that collect shared code
statistics from the OpenStatistic endpoint (`enb-01/probe-srslte-enb.xml`):

```XML
<probe-srslte-enb address="127.0.0.1" port="47100">
  <probes>
    <SRSLTE.ENB.Tables.Counts enable="yes"/>
    <SRSLTE.ENB.Tables.MAC enable="yes"/>
    <SRSLTE.ENB.Tables.PHY enable="yes"/>
    <SRSLTE.ENB.Tables.RLC enable="yes"/>
    <SRSLTE.ENB.Tables.Upper enable="yes"/>
  </probes>
</probe-srslte-enb>
```

and probes that collect radio model statistics from the Control Port
(`enb-01/probe-emanelte-enb.xml`):

```XML
<probe-emane-lte address="127.0.0.1" port="47000">
  <probes>
    <EMANE.LTE.ENB.Counters.General enable="yes"/>
    <EMANE.LTE.ENB.Tables.Counts enable="yes"/>
    <EMANE.LTE.ENB.Tables.Downlink.Control enable="no"/>
    <EMANE.LTE.ENB.Tables.Downlink.Data enable="no"/>
    <EMANE.LTE.ENB.Tables.Events enable="yes"/>
    <EMANE.LTE.ENB.Tables.Uplink.Control enable="no"/>
    <EMANE.LTE.ENB.Tables.Uplink.Data enable="no"/>
  </probes>
</probe-emane-lte>
```                        

Query the `otestpointd` discovery endpoint for a list of running probes:

```
[me@host two_ues]$ otestpoint-discover lxc-enb-01:8881
tcp://0.0.0.0:8882
EMANE.LTE.ENB.Counters.General.enb-01
EMANE.LTE.ENB.Tables.Counts.enb-01
EMANE.LTE.ENB.Tables.Events.enb-01
SRSLTE.ENB.Tables.Counts.enb-01
SRSLTE.ENB.Tables.MAC.enb-01
SRSLTE.ENB.Tables.PHY.enb-01
SRSLTE.ENB.Tables.RLC.enb-01
SRSLTE.ENB.Tables.Upper.enb-01
```

Inspect current probe values from the publish endpoint:

```
[me@host two_ues]$ otestpoint-dump lxc-enb-01:8882 SRSLTE.ENB.Tables.RLC

[1548773090] enb-01/0 9fec768d-185d-456e-9a47-b811fa83f234
otestpoint.lte.srsenb Measurement_srslte_enb_tables_rlc v1 1077 bytes
SRSLTE.ENB.Tables.RLC.enb-01
[] rlcmrbthruputtable
|RNTI|LCID|DLKbps|Type|Cap|Size|HighWater|NumPush|NumPushFail|NumPop|NumPopFail|Cleared|Time|
--
[] rlcthruputtable
|RNTI|LCID|DLKbps|ULKbps|Type|Cap|Size|HighWater|NumPush|NumPushFail|NumPop|NumPopFail|Cleared|Time      |
|70  |3   |0.0   |0.0   |UM  |128|0   |0        |0      |0          |0     |0         |0      |1548773089|
|70  |2   |0.0   |0.0   |AM  |128|0   |0        |0      |0          |0     |0         |0      |1548773089|
|70  |1   |0.0   |0.0   |AM  |128|0   |1        |5      |0          |5     |0         |0      |1548773089|
|70  |0   |0.0   |0.0   |TM  |16 |0   |1        |1      |0          |1     |0         |0      |1548773089|
|71  |1   |0.0   |0.0   |AM  |128|0   |1        |5      |0          |5     |0         |0      |1548773089|
|71  |0   |0.0   |0.0   |TM  |16 |0   |1        |1      |0          |1     |0         |0      |1548773089|
|71  |3   |0.0   |0.0   |UM  |128|0   |0        |0      |0          |0     |0         |0      |1548773089|
|71  |2   |0.0   |0.0   |AM  |128|0   |0        |0      |0          |0     |0         |0      |1548773089|
```


Stop and clean the demo.

```
[me@host two_ues]$ letce2 lxc stop
host is not running
running host stop.local
[me@host two_ues]$ letce2 lxc clean
```
---

## ENB and UE Radio Model Configuration Parameters

The ENB and UE Radio Model accept the same set of configuration parameters.

* [maxpropagationdelay](\#maxpropagationdelay)
* [pcrcurveuri](\#pcrcurveuri)
* [resourceblocktxpower](\#resourceblocktxpower)


### **maxpropagationdelay**
LTE defines a timing advance measurement reported from the ENB to each
UE. Each UE advances its transmit frame timing relative to its receive
frame timing based on the measurement, so that all UE transmissions
are received, roughly, coincidentally at the ENB, irrespective of UE
distance from the ENB. The LTE timing advance measurement numeric range
can correct for UE distances up to a maximum of 100 kilometers from the ENB.

Currently, EMANE LTE does not implement dynamic timing
advance. Propagation delays reported by the EMANE PHY for RF
receptions are ignored for the purpose of calculating the start of
reception time, essentially resulting in 0 propagation delay and
perfect timing advance.  However to avoid allowing arbitrarily large
cell sizes, receive processing drops any receptions with
progagation delay greater than the maxpropagationdelay (microseconds)
setting. The default value, 0, disables this check.

*Type:* uint64    
*Running-State Modifiable:* no   
*Occurrence Range:* [0,1]   
*Value Range:* [0,18446744073709551615]    
*Default Value:* 0    


### **pcrcurveuri**
Defines the URI of the LTE Packet Completion Rate (PCR) curve
file. The PCR curve file contains probability of reception curves as a
function of Signal to Interference plus Noise Ratio (SINR).

*Type:* string     
*Running-State Modifiable:* no   
*Occurrence Range:* [1,1]    


### **resourceblocktxpower**
The transmit power per LTE Resource Block (dBm).

*Type:* float    
*Running-State Modifiable:* no   
*Occurrence Range:* [0,1]   
*Value Range:* [1.17549e-38,3.40282e+38]    
*Default Value:* 0.0    
