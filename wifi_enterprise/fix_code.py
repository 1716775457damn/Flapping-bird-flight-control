import re

# 读取文件
with open('main/car_control_main.c', 'r', encoding='utf-8') as f:
    content = f.read()

# 找到pid_reset函数和test_pin13_pwm函数
pid_reset_pattern = r'static void pid_reset\(pid_controller_t \*pid\) \{[^}]+\}'
test_pin13_pattern = r'static void test_pin13_pwm\(void\) \{(?:[^{}]|(?:\{[^{}]*\}))*\}'

# 查找pid_reset函数
pid_reset_match = re.search(pid_reset_pattern, content)
if not pid_reset_match:
    print("无法找到pid_reset函数")
    exit(1)

# 查找test_pin13_pwm函数
test_pin13_match = re.search(test_pin13_pattern, content)
if not test_pin13_match:
    print("无法找到test_pin13_pwm函数")
    exit(1)

# 提取test_pin13_pwm函数的完整实现
test_pin13_function = test_pin13_match.group(0)
print(f"找到test_pin13_pwm函数:\n{test_pin13_function[:50]}...")

# 删除原始test_pin13_pwm函数实现
content_without_test_pin13 = content.replace(test_pin13_function, '')

# 在pid_reset后插入test_pin13_pwm函数
pid_reset_end = pid_reset_match.end()
new_content = content_without_test_pin13[:pid_reset_end] + "\n\n" + test_pin13_function + "\n" + content_without_test_pin13[pid_reset_end:]

# 写入文件
with open('main/car_control_main.c', 'w', encoding='utf-8') as f:
    f.write(new_content)

print("文件已更新，test_pin13_pwm函数已移动到pid_reset函数之后") 