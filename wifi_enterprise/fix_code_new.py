import re

# 读取原始文件
with open('main/car_control_main.c.original', 'r', encoding='utf-8') as f:
    original_content = f.read()

# 提取test_pin13_pwm函数的实现
test_pin13_pattern = r'// 添加直接测试13号引脚的函数，方便调试\nstatic void test_pin13_pwm\(void\) \{.*?\n\}'
test_pin13_match = re.search(test_pin13_pattern, original_content, re.DOTALL)
if not test_pin13_match:
    print("无法找到test_pin13_pwm函数")
    exit(1)

test_pin13_function = test_pin13_match.group(0)
print(f"找到test_pin13_pwm函数, 长度: {len(test_pin13_function)}")

# 删除原始文件中的test_pin13_pwm函数实现
content_without_test_pin13 = re.sub(test_pin13_pattern, '', original_content, flags=re.DOTALL)

# 找到pid_reset函数
pid_reset_pattern = r'static void pid_reset\(pid_controller_t \*pid\) \{[^}]+\}'
pid_reset_match = re.search(pid_reset_pattern, content_without_test_pin13)
if not pid_reset_match:
    print("无法找到pid_reset函数")
    exit(1)

# 在pid_reset函数后插入test_pin13_pwm函数
pid_reset_end = pid_reset_match.end()
new_content = content_without_test_pin13[:pid_reset_end] + "\n\n" + test_pin13_function + "\n" + content_without_test_pin13[pid_reset_end:]

# 写入新文件
with open('main/car_control_main.c', 'w', encoding='utf-8') as f:
    f.write(new_content)

print("文件已更新: main/car_control_main.c") 