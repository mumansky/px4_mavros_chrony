# px4_mavros_chrony
A simple MAVROS to chrony forwarder for synchronizing an FCU to a companion computer.

## Prequisites and Assumptions
* [mavros](https://github.com/mavlink/mavros) is installed and running on your host machine connected to a MAVLINK compliant autopilot
* `/mavros/time_reference` is the ROS2 topic
* Your autopilot has a GPS lock and valid timesync

## Installation
1. Install sysv_ipc
````pip install sysv_ipc````

2. Add SHM (shared host memory) source to /etc/chrony.conf as preferred:
```refclock SHM 0 poll 3 refid PX4 prefer```

3. Restart chronyd service
```sudo systemctl restart chronyd```

4. Run the python program 
```python3 sync_mavros_to_chrony.py```

5. Verify your chrony services in a seperate terminal

Check `chronyc sources -v` to verify your `PX4` source appears in your list. Example below:

```
  .-- Source mode  '^' = server, '=' = peer, '#' = local clock.
 / .- Source state '*' = current best, '+' = combined, '-' = not combined,
| /             'x' = may be in error, '~' = too variable, '?' = unusable.
||                                                 .- xxxx [ yyyy ] +/- zzzz
||      Reachability register (octal) -.           |  xxxx = adjusted offset,
||      Log2(Polling interval) --.      |          |  yyyy = measured offset,
||                                \     |          |  zzzz = estimated error.
||                                 |    |           \
MS Name/IP address         Stratum Poll Reach LastRx Last sample               
===============================================================================
#* PX4                           0   3   377     6   +146us[ +211us] +/-  194us
^- prod-ntp-4.ntp1.ps5.cano>     2   6    77    54    +15ms[  +15ms] +/-   48ms
^- prod-ntp-3.ntp4.ps5.cano>     2   6    77    55    +15ms[  +16ms] +/-   48ms
^- prod-ntp-5.ntp4.ps5.cano>     2   6    77    57    +17ms[  +17ms] +/-   48ms
^- alphyn.canonical.com          2   6    77    57    +16ms[  +16ms] +/-   38ms
```

Check `chronyc tracking` to verify chrony is using the PX4 source

```
Reference ID    : 50583400 (PX4)
Stratum         : 1
Ref time (UTC)  : Mon May 26 02:40:44 2025
System time     : 0.000120130 seconds fast of NTP time
Last offset     : +0.000290982 seconds
RMS offset      : 0.005652302 seconds
Frequency       : 4.261 ppm slow
Residual freq   : -13.345 ppm
Skew            : 1.228 ppm
Root delay      : 0.000000001 seconds
Root dispersion : 0.000728063 seconds
Update interval : 8.0 seconds
Leap status     : Normal
```

# What this doesn't do (yet)?
* Check for valid time sync
* Handle any kind of interruptions
* Handle no GPS startup or GPS loss situations