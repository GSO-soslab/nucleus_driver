# 4 Basic Operation
This section explains how to get started with some of the core functions of the Nucleus. Some functions might not be available since they require an additional license to be used. Licenses can be changed from the Licenses tab under "Instrument > Maintenance > Licenses".

## 4.1 Measuring Velocity - DVL
One of the fundamental features of the Nucleus is its ability to measure velocity. The instrument can determine its own velocity in relation to the seabed/bottom, which is referred to as Bottom Track (BT) velocity. Additionally, it is capable of measuring its velocity relative to the water, known as Water Track (WT). This operates by pinging off the particles in the water and the measurement volume is typically a few meters from the Nucleus. Water track is less accurate than bottom track, but useful when the bottom is out of range.

All velocity measurements are obtained by emitting an acoustic pulse from the slanted transducers and estimating the Doppler shift in the return pulse. The instrument provides velocity outputs along the beam axes, known as beam velocities, as well as in an XYZ coordinate system fixed within the instrument.

**Quick set-up:**

| Step | Command | Software | Comment |
| :--- | :--- | :--- | :--- |
| 1 | `SETBT, MODE = "FAST_ACQ" / "AUTO" / "CRAWLER"` | - | See information about Tracking Mode below |
| 2 | `SETBT,DS="ON"` | Go to Instrument/Configuration Check Bottom Track data stream | Ensure BT data is transmitted |
| 3 | `SETBT,WT="ON"` | Go to Instrument/Configuration Check Water Track data stream | Set this if WT data is wanted |
| 4 | `START` | Press Play button | Start measurement |
| 5 | `STOP` | Press Stop button | Stops measurement |

*Table 5: Bottom and Water track setup via command line*

Several additional arguments can also be set, to adjust the behavior of the instrument.

**Tracking mode**
The bottom detection mode allows for three different options for detecting the bottom. The modes are as follows:

**Fast_Acq mode**
This mode is intended for general Nucleus use and is used for the Nucleus' full range of distances from the bottom as well as the full range of velocities. It is the bottom track legacy mode of the Nortek Nucleus.

**Crawler mode**
The crawler mode is for aiding vehicles that intend to operate in station keeping mode or move very slowly. The advantage of the Crawler mode is that it has lower uncertainty in the velocity estimates, but is limited in range and beam velocity. The minimum detectable distance is 0.1 m and the maximum is 10 m. The default beam velocity limit is 0.15 m/s, which defines the velocity range (VR). The velocity range spans from -VR to +VR. The beam velocity limit ranges from [0.05, 0.4] m/s. Bottom track estimates exceeding the beam velocity limit gets ignored. The horizontal velocity range is approximately 2.9 times greater than the beam velocity range which translates to a maximum velocity of 0.44 m/s for the default beam velocity.

**Auto mode**
Auto mode is a hybrid mode that consists of Fast_Acq and Crawler.. As the name suggests, the Nucleus will automatically change between Fast_Acq and Crawler based on distance to the bottom and the measured velocity. In Auto mode, the Nucleus will switch to Crawler mode after a consistent detection of 10 consecutive pings with an along-beam velocity of 0.1 m/s and a bottom distance below 7.5 meters. To handle accelerations, the Nucleus will switch back to Fast_Acq mode on the first detection of an along-beam velocity above the 0.1 m/s threshold. It will also switch back to Fast_Acq mode after 4 consecutive missed detections or 4 detections above the before mentioned range limit.

### 4.1.1 Water track
**NB!** Enabling/disabling the water track measurement completely is done with `SETBT,WT`. Choosing how and whether the measurements get used for INS estimates are done with `SETWT` and `SETNAV`.

Velocity estimates from Water Track (WT) are only available when the vertical range beneath the Nucleus is at least 2 meters. As the vertical range increases, the WT cell gets larger.

In the figure below, the top sub figure describes where the water cell starts and stops, based on the vertical range beneath the Nucleus. There is no measurements if the vertical range is less than 2 meters. As the vertical range increases the WT cell starts further way from Nucleus, until it flattens out, same goes for where it stops.

The bottom sub figure shows the delta between WT start and stop as the vertical range increases.

[Image: Water Track Nucleus 1000 graphs showing WT cell vertical position and size vs. vertical range.]

*Figure 10: Top subfigure: Illustrates where the WT cell starts and stops. Bottom subfigure: Shows how the WT cell increases in size.*

**Water track mode - FIXED**
For areas with predictable current patterns and/or access to external measurements, it will be beneficial to use set the mode to FIXED. This will assume that the velocity and direction of the current is fixed, then provide a velocity estimate for the Nucleus based on water track measurements. The velocity estimate will be used to estimate the INS data. See `UPDATEWT,CUR` for how to update current velocity and direction during runtime.

**Water track mode - ESTCUR**
For areas with unpredictable current patterns and/or the lack of external current measurements, you can use ESTCUR. This will estimate the direction and velocity of the current, then provide a velocity estimate for the Nucleus based on watertrack measurements. The velocity estimate will be used to esitmate INS data.

## 4.2 Orientation Estimates - AHRS
The AHRS (Attitude and Heading Reference System) is a vital component of the Nucleus instrument, providing accurate measurements of orientation. It combines accelerometers, gyroscopes and magnetometers to determine the instrument's orientation in three-dimensional space. The AHRS calculates pitch, roll and heading angles, enabling precise navigation, motion tracking, and stabilization. Orientation is given in pitch, roll, and heading, as DCM (Direction Cosine Matrix), and as quaternion. The heading is based on magnetic measurements and aided by gyroscopes.

**Quick set-up**
Unless you are aware that you have significant disturbances in your magnetic measurements, we advise to first try setting up the measurement as if it has low levels of disturbances:

**Quick set-up (low noise)**
Calibrate your magnetic compass, if you haven't done so already. Make sure it is mounted on the vehicle as it will be used.

| Step | Command | Software | Comment |
| :--- | :--- | :--- | :--- |
| 1 | `SETFIELDCAL, MODE = 1` | Go to the compass calibration page and select hard iron | Set up calibration with hard iron only |
| 2 | `FIELDCAL` | Press the start button | Starts the calibration |
| 3 | | | Rotate your instrument |
| 4 | `STOP` | Press Stop | Stops the calibration and measurement |
| 5 | `GETMAGCAL` | | Returns calibrated hard iron parameters |
| 6 | `SAVE,MAGCAL` | Press Save Calibration | Saves hard iron parameters and reset soft iron |

*Table 6: Low noise compass calibration*

Perform a regular measurement to check if your system performs acceptable:

| Step | Command | Software | Comment |
| :--- | :--- | :--- | :--- |
| 1 | `SETAHRS,MODE=0` | Go To Instrument/Configuration Set AHRS estimation mode = Fixed | Use fixed hard and soft iron values |
| 2 | `START` | Press Play button | Start measurement |
| 3 | | | Perform motion |
| 4 | `STOP` | Press Stop button | Stop Measurements |
| 5 | | | Review performance |

*Table 7: Check calibration performance steps*

If performance is acceptable, the MAGCAL values are already saved and will be used next time instrument is powered on. The instrument can be used as in the regular measurement explained above. If performance is not within expectations, this may be due to:

a) Poor calibration. Re-do the calibration, and make sure your motion is smooth and covers all/many orientations
b) System requires compensation of soft iron also. Re-do calibration, but use `SETFIELD,MODE = 2` (selects hard and soft iron calibration).
c) There is too much noise. The assumption that hard and soft iron can be compensated with static values fails. Try setting up the system as in the "high noise" case described below.

**Quick set-up (high noise)**
Make sure the instrument is mounted on the vehicle it should be used. Perform a regular measurement to check if your system performs acceptable:

| Step | Command | Software | Comment |
| :--- | :--- | :--- | :--- |
| 1 | `SETAHRS, MODE = 1` | Go To Instrument/Configuration Set AHRS estimation mode = Hard iron | Continuous estimation of hard iron |
| 2 | `START` | Press Play button | Start measurement |
| 3 | | | Make sure to rotate the instrument a few rounds in all possible directions to allow good initialization |
| 4 | | | Perform operation |
| 5 | `STOP` | Press Stop button | Stop measurements |
| 6 | | | Review performance |

*Table 8: Steps for continuous hard iron parameter estimation during operation*

If performance is acceptable, it is encouraged to save the last estimates of hard and soft iron (`SAVE,MAGCAL`). This can assist with shorter start-up time next time. The instrument can be used as in the regular measurement explained above. If performance is not within expectations, this may be due to:

a) Poor estimation. Make sure instrument motion is sufficient to observe the compensation parameters (hard and soft iron). This is especially true in the start-up phase.
b) System requires compensation of soft iron also. Re-run measurement and check performance, but use `SETAHRS, MODE = 2` (continuous estimation of hard and soft iron), or if you use the Software: Go to Instrument/Configuration, and set AHRS estimation mode = Hard and soft iron.
c) The amount of magnetic disturbance is too high. Review design of vehicle and placement of instrument, focusing on reducing magnetic disturbance.

**Magnetic declination**
Setting magnetic declination is recommended to ensure that output heading is better aligned to true north. If heading should be aligned to magnetic north, magnetic declination should be set to zero.

*   Setting declination to a fixed value:
    1.  `SETMISSION, DECL = xx` (sets declination angle)
    2.  `SETMAG, METHOD = "OFF"` (ensures that the instruments selects the `SETMISSION,DECL` angle)
*   Setting declination based on the World Magnetic Map (WMM):
    1.  `SETMAG, METHOD = "AUTO"` (alternatively `METHOD = "WMM"`)
    2.  `SETMISSION, LAT = yy, LONG = zz` (WMM requires knowledge of position)
    3.  `SETCLOCKSTR, TIME = "yyyy-mm-dd hh:mm:ss"` (Not required, but knowledge of time improves accuracy of WMM)

**Calibration modes**
AHRS settings in the Commands chapter outlines the three different modes (0, 1 or 2) for utilizing the AHRS, based on the desired calibration against different types of iron (Hard/Soft) and their fixed or variable values. It is important to note than when operating in the fixed hard/soft iron mode (MODE=0), a field calibration should be performed before deployment. In cases where iron levels are expected to vary (MODE=1 or 2), the calibration becomes adaptive. In this scenario, it is necessary to drive the Nucleus in a few circles before operation, allowing the instrument to automatically calibrate itself. Engaging in any driving activity prior to this calibration may result in offsets and unreliable data.

## 4.3 Position Estimates - Navigation
The Nucleus incorporates various sensors that work in conjunction to offer precise estimations of the instrument's movements underwater. By understanding and implementing the correct operational procedures and ensuring the appropriate license is installed, you can rely on the Nucleus to provide real-time positional estimates.

**Position**
The Nucleus can provide position estimates using two coordinate systems: Latitude/Longitude, which specifies the location on the Earth's surface, and a local NED coordinate system, which denotes North, East, and Down directions. This local coordinate system provides a relative position from a known start position.

**Quick set-up**
The instrument will always estimate the local NED position. It will assume it starts at X= 0, Y = 0. However, to estimate the Latitude/Longitudinal position, the instrument must be provided with a starting point:

| Step | Command | Software | Comment |
| :--- | :--- | :--- | :--- |
| 1 | `SETINST,TYPE="NAV"` | Go to Instrument/Configuration Set Instrument mode = "Navigation" | Enables position estimates Requires INS-license |
| 2 | `SETMISSION,LONG=xx,LAT=yy` (`SETMISSION,DECL=zz`) | Select advanced mode (ctrl+alt+n) Go to Instrument/Mission Set Longitude, Latitude, or use map (Set Magnetic declination, or use map) | Setting longitude and latitude is required to have valid Long-Lat output. This also makes it possible to estimate magnetic declination based on World Magnetic Mag (see `SETMAG`) |
| 3 | `SETAHRS,MODE={0,1,2}` | Go to Instrument/Configuration Set AHRS estimation mode. | Depends on use, and properties of the vehicle, see "Orientation Estimates - AHRS" |
| 4 | `SETNAV,DS="ON"` | Go to Instrument/Configuration Check AHRS data stream | Ensures INS data is transmitted |
| 5 | `START` | Press the Play button | Start your measurements. The filter requires some time to settle, and during which the position estimates may drift |
| 6 | `UPDATEPOS,argument=` | Go to Dashboard/INS Select new position in map, and hit "update vehicle position" | If needed, this will update, or move, the current position. Updating X,Y will not update LongLat, and vice versa. They all have to be updated explicitly. This can be applied any time during measurement. |
| 7 | `STOP` | Press Stop button | Stops measurement |

*Table 9: Navigation mode setup for Nucleus 1000*

Several additional arguments can also be set to adjust the behavior of the instrument. See Commands for more information.

**Assumptions and sources of error**
It is well-known that relying solely on inertial sensors to estimate velocity leads to a growing error in velocity estimate. In turn, this leads to an even faster-growing error in position estimate. Therefore, the DVL velocity measurements are crucial to improve velocity estimate, and also the position estimate. However, there may be situations, such as when distance to the seabed is too large, or when velocity measurements will be lost over a period of time. In such cases, the instrument will in the very short term rely on the inertial measurements, but quite fast, it will switch mode, and start to enforce a decaying velocity. Lacking knowledge of true velocity, this imposed decaying velocity, may be quite wrong, and can become a significant source of error in the position estimate.

In addition to erroneous or inaccurate velocity measurements, there are other significant sources to the positional error to consider. One major contributor is heading. Any error in the heading measurements will propagate and result in positional inaccuracies. Heading errors can arise from various sources, please see “Orientation Estimates - AHRS" for more information on this.

Magnetic declination is set at the START of each measurement. Using `UPDATEPOS` to adjust LatLong coordinates while in measurement mode, does not update magnetic declination (even if WMM is used to find declination). Neither is the magnetic declination updated when moving over shorter or longer distances. The magnetic declination is assumed fixed throughout a measurement.

## 4.4 Doing Multiple Acoustic Measurements - DVL/Altimeter/Current Profile
Multiple acoustic measurements can be conducted simultaneously, utilizing the Nucleus's DVL, Altimeter, and Current Profiling functions. These measurements provide valuable data on velocity, depth, and water currents; this enables a comprehensive understanding of the vehicle's relation to the underwater environment.

| Step | Command | Comment |
| :--- | :--- | :--- |
| 1 | `SETBT,MODE="FAST_ACQ"/"AUTO"/"CRAWLER"` | Set up the Bottom Track |
| 2 | `SETALTI,DS="ON"` | Enable data stream of Altimeter |
| 3 | `SETCURPROF,DS="ON"` | Enable data stream of Current Profile |
| 4 | `SETBT,WT="ON"` | Enable Water Track, if wanted Note that no water track is performed while in "CRAWLER" mode |
| 5 | `SETBT,DS="ON"` | Ensure data is streamed |
| 6 | `SETTRIG,SRC="INTERNAL",FREQ=2,ALTI=xx,CP=yy` | Schedule the BT/Altimeter/Current profile pings Here xx and yy is the interleave ratio of pings relative to the baseline ping which is the Bottom Track |
| 7 | `START` | Start your measurements and estimations |

*Table 10: Multiple acoustic measurements setup*

Several additional arguments can also be set to adjust the behavior of the instrument. See Commands for more information.

**Additional considerations for current profiles**
When using the current profile option, one will see that the current profile is composed of a series of range bins with an independent estimate of current at each range. The extent of the profile is determined by the start (Blanking distance), spatial resolution (Cell size), and end of the profile (Range).

The estimates from the current profile are based on single pings and are provided in the coordinate system of the Nucleus, which is X, Y, Z. The current profiles are in the instruments frame of reference, so any velocity of the vehicle will be included in the estimate. If an Earth frame of reference is desired, then the vehicles velocity as well as attitude need to be accounted for.

Current profile estimates also include measures of amplitude and correlation. These may be used to quality control the estimates. Estimates that have either low amplitude (typically 26 dB for noise free environments) or Correlation below 50% should be discarded.

---

# 7.1 List of Commands

Below is a list of all available commands with a short description and information about which mode they can be used in.

| Command | Description | Mode |
| :--- | :--- | :--- |
| **START** | Start measurement | COMMAND |
| **STOP** | Stop measurement. | MEASUREMENT |
| **TRIG** | Trigger an acoustic measurement | MEASUREMENT |
| **FIELDCAL** | Start field calibration procedure | COMMAND |
| **SETFASTPRESSURE** | Set fast pressure settings | COMMAND |
| **GETFASTPRESSURE** | Get fast pressure settings | COMMAND |
| **GETFASTPRESSURELIM** | Get fast pressure setting limits | COMMAND |
| **SAVE** | Save active settings | COMMAND |
| **SETDEFAULT** | Revert to default settings | COMMAND |
| **RESTORE** | Restore settings from saved values | COMMAND |
| **SETMISSION** | Set mission settings | COMMAND |
| **GETMISSION** | Get mission settings | COMMAND |
| **GETMISSIONLIM** | Get limits for mission settings | COMMAND |
| **SETINST** | Set instrument settings | COMMAND |
| **GETINST** | Get instrument settings | COMMAND |
| **GETINSTLIM** | Get limits for instrument settings | COMMAND |
| **SETAHRS** | Set AHRS settings | COMMAND |
| **GETAHRS** | Get AHRS settings | COMMAND |
| **GETAHRSLIM** | Get limits for AHRS settings | COMMAND |
| **SETNAV** | Set navigation settings | COMMAND |
| **GETNAV** | Get navigation settings | COMMAND |
| **GETNAVLIM** | Get limits for navigation settings | COMMAND |
| **SETFIELDCAL** | Set field calibration settings | COMMAND |
| **GETFIELDCAL** | Get field calibration settings | COMMAND |
| **GETFIELDCALLIM** | Get limits for field calibration settings | COMMAND |
| **APPLYNAV** | Apply NAV settings during mission. This command is only valid after START command. | MEASUREMENT |
| **APPLYNAVLIM** | Get limits for APPLYNAV. | COMMAND |
| **SETBT** | Set bottom track settings | COMMAND |
| **GETBT** | Get bottom track settings | COMMAND |
| **GETBTLIM** | Get limits for bottom track settings | COMMAND |
| **APPLYTAG** | Apply a tag to the dataset during mission. This command is only valid after START command. | MEASUREMENT |
| **SETWT** | Set water track settings | COMMAND |
| **GETWT** | Get water track settings | COMMAND |
| **GETWTLIM** | Get water track setting limits | COMMAND |
| **SETALTI** | Set altimeter settings | COMMAND |
| **GETALTI** | Get altimeter settings | COMMAND |
| **GETALTILIM** | Get limits for altimeter settings | COMMAND |
| **SETCURPROF** | Set current profile settings | COMMAND |
| **GETCURPROF** | Get current profile settings | COMMAND |
| **GETCURPROFLIM** | Get limits for current profile settings | COMMAND |
| **SETTRIG** | Set trigger settings | COMMAND |
| **GETTRIG** | Get trigger settings | COMMAND |
| **GETTRIGLIM** | Get limits for trigger settings | COMMAND |
| **SETIMU** | Set IMU settings | COMMAND |
| **GETIMU** | Get IMU settings | COMMAND |
| **GETIMULIM** | Get limits for IMU settings | COMMAND |
| **SETMAG** | Set magnetometer settings | COMMAND |
| **GETMAG** | Get magnetometer settings | COMMAND |
| **GETMAGLIM** | Get limits for magnetometer settings | COMMAND |
| **SETMAGCAL** | Set magnetometer calibration values | COMMAND |
| **GETMAGCAL** | Get magnetometer calibration values | COMMAND |
| **GETMAGCALLIM** | Get limits for magnetometer calibration settings | COMMAND |
| **UPDATEPOS** | Update local position during mission. This command is only valid after START command. | MEASUREMENT |
| **UPDATEWT** | Update how Water Track measurements are included in NAV estimation and current velocity during mission. This command is only valid after START command. | MEASUREMENT |
| **UPDATEWTLIM** | Get limits for UPDATEWT. | COMMAND |
| **SETETH** | Set Ethernet settings | COMMAND |
| **GETETH** | Get Ethernet settings | COMMAND |
| **GETETHLIM** | Get limits for Ethernet settings | COMMAND |
| **READIP** | Read IP address | COMMAND |
| **GETERROR** | Returns a full description of the last error condition to occur | COMMAND |
| **ID** | Get instrument Id | COMMAND |
| **GETHW** | Get board revisions | COMMAND |
| **GETFW** | Get firmware version | COMMAND |
| **SETCLOCKSTR** | Set instrument clock as string | COMMAND |
| **GETCLOCKSTR** | Get instrument clock as string | COMMAND |
| **GETALL** | Retrieves all relevant configuration information for the instrument | COMMAND |
| **REBOOT** | Reboot the instrument | COMMAND |
| **LISTLICENSE** | List all license keys in the instrument | COMMAND |
| **ADDLICENSE** | Add license key | COMMAND |
| **DELETELICENSE** | Delete license key | COMMAND |