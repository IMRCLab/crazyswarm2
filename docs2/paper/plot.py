import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches

labels = []
def add_label(violin, label):
    color = violin["bodies"][0].get_facecolor().flatten()
    labels.append((mpatches.Patch(color=color), label))


if __name__ == '__main__':
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(9, 4))
    ax.set_xlabel("Number of Robots")
    ax.set_ylabel("Latency [ms]")

    for backend, label in zip(["cpp", "cflib", "nbc_cpp", "nbc_cflib"],["C++ (mocap)", "cflib (mocap)", "C++ (no broadcasts)", "cflib (no broadcasts)"]):

        all_latencies = []
        for num_robots in range(1, 9):
            # print(num_robots)
            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(
                uri='{}cfs_{}'.format(num_robots, backend),
                storage_id='mcap')
            converter_options = rosbag2_py.ConverterOptions('', '')
            reader.open(storage_options, converter_options)

            topic_types = reader.get_all_topics_and_types()

            # Create a map for quicker lookup
            type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

            latencies = []
            while reader.has_next():
                (topic, data, t) = reader.read_next()
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)

                if "status" in topic:
                    latencies.append(msg.latency_unicast)
                    # print(topic, msg.latency_unicast)
                if len(latencies) >= num_robots * 20:
                    break
            print(num_robots, latencies)
            all_latencies.append(latencies)



        all_data = all_latencies


        # plot violin plot
        add_label(ax.violinplot(all_data,
                        showmeans=False,
                        showmedians=True), label)

        # axs[0].set_title('Violin plot')

        # # plot box plot
        # axs[1].boxplot(all_data)
        # axs[1].set_title('Box plot')

        # # adding horizontal grid lines
        # for ax in axs:
        #     ax.yaxis.grid(True)
        #     ax.set_xticks([y + 1 for y in range(len(all_data))],
        #                 labels=['x1', 'x2', 'x3', 'x4'])
        #     ax.set_xlabel('Four separate samples')
        #     ax.set_ylabel('Observed values')

    plt.legend(*zip(*labels), loc=2)
    plt.show()