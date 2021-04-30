#!/bin/bash
iverilog -g2012 -o etarget.out -c etarget.cmd
vvp etarget.out
