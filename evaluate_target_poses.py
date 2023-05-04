import os
import json
import glob
import numpy as np
import matplotlib.pyplot as plt

def load_json_files():
    json_files = glob.glob('*_traj.json')
    data = []

    for file in json_files:
        with open(file, 'r') as f:
            data.append(json.load(f))
            
    return data

def compute_statistics(data):
    stats = {'heading': {'average': [], 'maximum': [], 'minimum': [], 'percentiles': []},
             'position': {'average': [], 'maximum': [], 'minimum': [], 'percentiles': []}}

    for item in data:
        actual_poses = item['actual']
        ref_poses = item['ref']
        heading_deviations = []
        position_deviations = []

        for actual, ref in zip(actual_poses, ref_poses):
            actual_last_pose = actual['path'][-1]
            ref_last_pose = ref['path'][-1]
            
            heading_deviation = actual_last_pose['ph'] - ref_last_pose['ph']
            heading_deviations.append(heading_deviation)
            
            position_deviation = np.sqrt((actual_last_pose['px'] - ref_last_pose['px'])**2 +
                                         (actual_last_pose['py'] - ref_last_pose['py'])**2)
            position_deviations.append(position_deviation)

        for stat_name, deviations in zip(['heading', 'position'], [heading_deviations, position_deviations]):
            stats[stat_name]['average'].append(np.mean(deviations))
            stats[stat_name]['maximum'].append(np.max(deviations))
            stats[stat_name]['minimum'].append(np.min(deviations))
            stats[stat_name]['percentiles'].append(np.percentile(deviations, [0, 25, 50, 75, 100]))

    return stats

def save_statistics_as_json(stats, filename='stats.json'):
    with open(filename, 'w') as f:
        json.dump(stats, f, indent=4)

def create_boxplots(stats):
    fig, axs = plt.subplots(1, 2, figsize=(10, 5))
    axs[0].boxplot([p['percentiles'] for p in stats['heading']])
    axs[0].set_title('Heading Deviation at End Position')
    axs[0].set_xlabel('Track')
    axs[0].set_ylabel('Deviation (degrees)')

    axs[1].boxplot([p['percentiles'] for p in stats['position']])
    axs[1].set_title('Position Deviation at End Position')
    axs[1].set_xlabel('Track')
    axs[1].set_ylabel('Deviation (meters)')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    data = load_json_files()
    stats = compute_statistics(data)
    save_statistics_as_json(stats)
    create_boxplots(stats)
