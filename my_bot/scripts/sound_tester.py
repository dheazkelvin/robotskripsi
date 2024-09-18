#!/usr/bin/env python3

import paramiko

def play_sound_on_raspberry_pi(host, port, username, password):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, port, username, password)

    command = "/home/kelvinyxe/Desktop/robot_ws/src/my_bot/scripts/play_sound.sh"
    stdin, stdout, stderr = ssh.exec_command(command)
    print(stdout.read().decode())
    print(stderr.read().decode())

    ssh.close()

# Example usage
raspberry_pi_ip = "172.20.10.3"
ssh_port = 22
ssh_username = "kelvinyxe"
ssh_password = "Nayonaise23"

# After the robot reaches the destination
play_sound_on_raspberry_pi(raspberry_pi_ip, ssh_port, ssh_username, ssh_password)
