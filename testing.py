a = ["a0","b0","c","t0"]

for file in a:
	print(file)

print("--------------")

for file in a:
	if "0" in file:
		print(file)

# a0
# b0
# c
# t0
# --------------
# a0
# b0
# t0

b= [[0,0,0,0],["a"," b","c","d"]]

print(len(b))

for line in b:
	if "a" in line:
		print("hey i found a")

# a0
# b0
# t0
# 2
# hey i found a