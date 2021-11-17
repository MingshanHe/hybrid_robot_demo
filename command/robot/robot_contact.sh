#!/usr/bin/expect
spawn ssh root@192.168.188.99
expect "root@192.168.188.99's password: "
send"root\r"
interact
