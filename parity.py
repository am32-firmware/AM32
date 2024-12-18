
word = 0x7fff
tmp = 0
for i in range(15):
    tmp = tmp^((word >> i) & 1)

print(tmp)
