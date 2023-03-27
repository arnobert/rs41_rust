# Calculate the values for Si4032 nominal carrier frequency registers
# (!) The chip is not fed by 30 MHz but 26 MHz (!)
# hbsel := 1, fb[4:0] := 0 -> results in 416..433 MHz
# Formula from datasheet derived:
# 
# f_c = f_TX[MHz] * 6400 - 2752000

hbsel = 1
fb = 0
f_TX = 432.20

f_c = (f_TX / ((26/3) * (hbsel + 1)) - fb - 24)  * 64000

print("Upper byte:")
print(hex(int(f_c) & 0xFF00))
print("Lower byte:")
print(hex(int(f_c) & 0x00FF))

