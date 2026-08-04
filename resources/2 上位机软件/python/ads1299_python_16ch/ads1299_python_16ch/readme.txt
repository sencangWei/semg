1、Python 修改 pip 源为国内源
https://www.cnblogs.com/137point5/p/15000954.html
临时换源：
#清华源
pip install markdown -i https://pypi.tuna.tsinghua.edu.cn/simple
# 阿里源
pip install markdown -i https://mirrors.aliyun.com/pypi/simple/
# 腾讯源
pip install markdown -i http://mirrors.cloud.tencent.com/pypi/simple
# 豆瓣源
pip install markdown -i http://pypi.douban.com/simple/

永久换源：
# 清华源
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
# 阿里源
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
# 腾讯源
pip config set global.index-url http://mirrors.cloud.tencent.com/pypi/simple
# 豆瓣源
pip config set global.index-url http://pypi.douban.com/simple/# 换回默认源pip config unset global.index-url

2、导出python环境中的依赖或导出项目中的依赖的几种方法
https://blog.csdn.net/qq_43617906/article/details/136564271
导出：pip list --format=freeze >requirement.txt 
导入：pip install -r requirements.txt

3、升级 pip
python -m pip install --upgrade pip
