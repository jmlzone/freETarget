yosys -s l40.ys
#arachne-pnr -d 1k -P swg16tr -p etarget.pcf etarget.blif -o etarget.asc
# 1k
#nextpnr-ice40 --lp1k --package swg16tr --verbose --top  etarget --json etarget.json --pcf etarget.pcf --asc etarget.asc --ignore-loops --freq 8
# 5k
nextpnr-ice40 --up5k --package sg48 --verbose --top  etarget --json etarget.json --pcf etargetu5k.pcf --asc etarget.asc --ignore-loops --pre-pack etarget_cons.py
icepack etarget.asc etarget.bin
#iceprog etarget.bin
#icetime -tmd lp1k etarget.asc
