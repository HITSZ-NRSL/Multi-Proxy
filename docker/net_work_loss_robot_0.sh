# tc qdisc add dev eth0 root handle 1: prio
# tc qdisc add dev eth0 parent 1:3 handle 30: netem loss 1% rate 0.5mbit delay 100ms
# tc filter add dev eth0 protocol ip parent 1:0 prio 3 u32 match ip dst 172.0.0.3/32 flowid 1:3
# tc filter add dev eth0 protocol ip parent 1:0 prio 3 u32 match ip dst 172.0.0.4/32 flowid 1:3
