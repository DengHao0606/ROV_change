import json
import os

pid_parameters_path = os.getcwd()+"/others/datas/pid_parameters.json"

pid = 0

with open(pid_parameters_path, 'r', encoding='utf-8') as fp:

    print(type(fp))  # 输出结果是 <class '_io.TextIOWrapper'> 一个文件类对象

    # load()函数将fp(一个支持.read()的文件类对象，包含一个JSON文档)反序列化为一个Python对象
    data = json.load(fp)
    pid = data
    # print(type(data))
    for p in data:
        print(p["p"])

# with open(pid_parameters_path, 'w', encoding='utf-8') as fp:

#     print(type(fp))  # 输出结果是 <class '_io.TextIOWrapper'> 一个文件类对象

#     # load()函数将fp(一个支持.read()的文件类对象，包含一个JSON文档)反序列化为一个Python对象
#     data = json.load(fp)

#     print(type(data))
#     print(data["x"]["p"])

# text = json.loads(data)
# print(text)


# load_data = json.loads(data)
# data = load_data.get("animals").get("dog")
# result1 = []
# for i in data:
#     result1.append(i.get("name"))
# print(result1)
