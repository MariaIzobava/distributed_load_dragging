import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time
import numpy as np

# URI to the Crazyflie to connect to
# Adjust this to your Crazyflie's address (e.g., 'radio://0/80/2M/E7E7E7E7E7')
URI = 'radio://0/81/2M/E7E7E7E7E7'

# Number of seconds to collect latency data
COLLECTION_DURATION_SECONDS = 10

def print_latency_stats(latency_values):
    """Prints statistics for the collected latency values."""
    if not latency_values:
        print("No latency data collected.")
        return

    mean_latency = np.mean(latency_values)
    median_latency = np.median(latency_values)
    std_dev_latency = np.std(latency_values)
    p95_latency = np.percentile(latency_values, 95)
    max_latency = np.max(latency_values)
    min_latency = np.min(latency_values)

    print("\n--- Latency Statistics ---")
    print(f"Total samples: {len(latency_values)}")
    print(f"Mean Latency: {mean_latency:.3f} ms")
    print(f"Median Latency: {median_latency:.3f} ms")
    print(f"Standard Deviation: {std_dev_latency:.3f} ms")
    print(f"95th Percentile Latency: {p95_latency:.3f} ms")
    print(f"Maximum Latency: {max_latency:.3f} ms")
    print(f"Minimum Latency: {min_latency:.3f} ms")

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    print(f'Connecting to {URI}')

    latency_data_points = []

    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        cf = scf.cf

        print('Crazyflie connected!')
        print(f'Collecting latency data for {COLLECTION_DURATION_SECONDS} seconds...')

        start_time = time.time()
        while time.time() - start_time < COLLECTION_DURATION_SECONDS:
            # The 'latency' attribute provides the 95th percentile of recent RTTs.
            # This value is continuously updated by cflib in the background.
            current_latency = cf.link_statistics.latency.latency
            
            if current_latency is not None:
                latency_data_points.append(current_latency)
                # print(f"Current 95th Percentile Latency: {current_latency:.3f} ms")
            
            time.sleep(0.01) # Sleep briefly to avoid busy-waiting

        print('Latency data collection complete.')
        print_latency_stats(latency_data_points)