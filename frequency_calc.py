# Calculate the values for Si4032 nominal carrier frequency registers
#
# hbsel := 0, fb[4:0] := 19 -> results in 70cm band
# Formula from datasheet derived:
# 
# f_c = f_TX[MHz] * 6400 - 2752000

f_TX = 430.00

f_c = f_TX * 6400 - 2752000

print("Upper byte:")
print(hex(int(f_c) & 0xFF00))
print("Lower byte:")
print(hex(int(f_c) & 0x00FF))

