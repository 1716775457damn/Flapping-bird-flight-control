import os
import requests
from bs4 import BeautifulSoup
import time

# 目标网站URL
url = 'https://thedigitaladda.com/free-certifications/'

# 发送HTTP请求并添加headers模拟浏览器
headers = {
    'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
}
response = requests.get(url, headers=headers)

# 解析网页内容
soup = BeautifulSoup(response.text, 'html.parser')

# 创建本地文件夹
folder_name = 'certificates'
if not os.path.exists(folder_name):
    os.makedirs(folder_name)

# 查找所有证书链接
certificates = soup.find_all('h2')

for certificate in certificates:
    # 获取链接元素
    link = certificate.find('a')
    if not link:
        continue
        
    # 获取证书名称和链接
    name = link.text.strip()
    cert_url = link['href']
    
    try:
        # 访问证书页面
        print(f'正在处理: {name}')
        cert_response = requests.get(cert_url, headers=headers)
        cert_soup = BeautifulSoup(cert_response.text, 'html.parser')
        
        # 查找页面中的特色图片
        img_tag = cert_soup.find('img', class_='wp-post-image')
        if not img_tag:
            # 尝试查找文章内容中的第一张图片
            img_tag = cert_soup.find('div', class_='entry-content').find('img')
            
        if img_tag and 'src' in img_tag.attrs:
            img_url = img_tag['src']
            
            # 下载图片
            img_response = requests.get(img_url, headers=headers)
            if img_response.status_code == 200:
                # 清理文件名（移除非法字符）
                safe_name = "".join([c for c in name if c.isalnum() or c in (' ', '-', '_')]).rstrip()
                file_path = os.path.join(folder_name, f'{safe_name}.jpg')
                
                with open(file_path, 'wb') as f:
                    f.write(img_response.content)
                print(f'已下载: {name}')
            else:
                print(f'下载失败: {name}')
        else:
            print(f'未找到图片: {name}')
            
        # 添加延时，避免请求过快
        time.sleep(1)
        
    except Exception as e:
        print(f'处理 {name} 时出错: {str(e)}')

print('所有证书已下载完成。')