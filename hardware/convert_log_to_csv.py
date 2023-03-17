import numpy as np
import cfusdlog
import argparse
# python3 path-to-log path-to-csv

def usdlog2csv(logfile, csvfile, use_mocap_data=False):
    data = cfusdlog.decode(logfile)
    # print(data)

    # sanity check on the quality of the data
    time_eP = data['estPose']['timestamp'] / 1e3
    dt_eP = np.diff(time_eP)
    rate_ep = 1/dt_eP
    print("Mocap rate: {:.1f} Hz ({:.1f})".format(np.mean(rate_ep), np.std(rate_ep)))

    if np.mean(rate_ep) < 75:
        print("WARNING: mocap rate is poor!", logfile)
        use_mocap_data=False
        # exit()

    # Notes on timing:
    # GAP sends data in ms and the datacollection script converts that to seconds
    # GAP8 synchronizes itself based on a "timesync event", which marks time 0
    # The timesync event happens when the cf_state timestamp (from the STM usec timer) suddenly gets smaller (i.e., is reset to 0)
    # The STM USEC timer gets reset to 0, on the first received broadcast localization event from the mocap
    # The usd log uses the STM USEC timer as a timestamp
    # Overall, the latency between GAP8 and STM should be < 10ms

    if use_mocap_data:
        logData = data['estPose']
        result = np.column_stack((
            logData['timestamp'] / 1000.0,   # this needs to be in secs
            logData['locSrv.x'],
            logData['locSrv.y'],
            logData['locSrv.z'],
            logData['locSrv.qw'],
            logData['locSrv.qx'],
            logData['locSrv.qy'],
            logData['locSrv.qz']))
    else:
        logData = data['fixedFrequency']
        result = np.column_stack((
            logData['timestamp'] / 1000.0,   # this needs to be in secs
            logData['stateEstimate.x'],
            logData['stateEstimate.y'],
            logData['stateEstimate.z'],
            logData['stateEstimate.qw'],
            logData['stateEstimate.qx'],
            logData['stateEstimate.qy'],
            logData['stateEstimate.qz']))

    with open(csvfile, "w") as file:
        file.write("image_name,timestamp,x,y,z,qw,qx,qy,qz\n")

        for row in result:
            t,x,y,z,qw,qx,qy,qz = row
            file.write("{},{},{},{},{},{},{},{},{}\n".format("None", t, x, y, z, qw, qx, qy, qz)) 


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    parser.add_argument("csvfile")
    args = parser.parse_args()
    usdlog2csv(args.logfile, args.csvfile)


if __name__ == "__main__":
    main()
