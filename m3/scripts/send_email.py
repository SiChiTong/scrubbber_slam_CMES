#!/usr/bin/python
# -*- coding: UTF-8 -
import sys
import os
import smtplib
import random
from email import encoders
from email.mime.text import MIMEText
from email.header import Header
from email.mime.multipart import MIMEMultipart, MIMEBase

# 收件人
mapping_email_group = ['gaoxiang@idriverplus.com',
                       'zhangqingbo@idriverplus.com',
                       'wangqi@idriverplus.com',
                       'guhao@idriverplus.com',
                       'panqingjun@idriverplus.com',
                       'sunyu@idriverplus.com',
                       'xuelina@idriverplus.com',
                       'shiqi@idriverplus.com',
                       'sunyu@idriverplus.com',
                       'weilei@idriverplus.com',
                       'genglei@idriverplus.com',
                       'weixingcai@idriverplus.com',
                       'chenchunguang@idriverplus.com',
                       'lishaoqian@idriverplus.com',
                       'herenjie@idriverplus.com',
                       'pengguoqi@idriverplus.com',
                       'siyiwen@idriverplus.com']

sender = 'mapping@idriverplus.com'
host = "smtphz.qiye.163.com"
port = 994
user = "mapping@idriverplus.com"
password = "aptx4869"

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print ('Usage: python send_email.py map_name map_data_folder')
        exit(1)
    map_name = sys.argv[1]
    map_folder = sys.argv[2]
    print ('map name=', map_name, 'map folder = ', map_folder)

    message = MIMEMultipart()
    message['From'] = sender
    message['To'] = "mapping group"
    message['Subject'] = Header('建图报告 of ' + map_name, 'utf-8')

    f = open(map_folder + '/task_info.txt')
    line = f.readline().split()
    valid_bags = line[2]
    area = line[5]
    build_status = line[6]
    check_in = line[9]
    if build_status == '1':
        build_result = '成功'
    elif check_in == '2':
        build_result = '准入失败'
    else:
        build_result = '待确认'
    simulation_status = line[8]

    if simulation_status == '0':
        simulation_result = '成功'
    elif simulation_status == '1':
        simulation_result = '人工质检'
    elif simulation_status == '2':
        simulation_result = '失败'

    start_time = f.readline()
    end_time = f.readline()

    zen_of_mapping = ['不积跬步，无以致千里。', '晓声长夜谈定位，杯酒阑干话地图。', '千里之行，始于足下。', '人不能两次踏入同一张地图。',
                      '细推物理须行乐，何用浮名伴此生。', '欲识他年画鼎处，先生笑指画图中。', '借问酒家何处是，牧童遥指杏花村。',
                      '不识庐山真面目，只缘身在此山中。', '只在此图中，云深不知处。', '别人笑我在建图，我笑别人建不了图。',
                      '爱上一朵花就陪她去绽放，爱上一个人就陪伴着她成长。', '不乱于心，不困于情，不畏将来，不念过往。']

    info = '任务名称/编号：' + map_name + '\n' + \
           '任务开始时间--结束时间：' + start_time + ' -- ' + end_time + '\n' + \
           '建图面积：' + area + ' 平方米，有效包数：' + valid_bags + '\n' + \
           '建图结果：' + build_result + ' 准出结果：' + simulation_result + '\n\n' + \
           zen_of_mapping[random.randint(0, len(zen_of_mapping) - 1)] + '\n' + \
           '----------------------------\n' + \
           '这是由Mapping 3.0自动发送的邮件，请勿回复.'

    message.attach(MIMEText(info, 'plain', 'utf-8'))

    # 附件
    print ('attaching report')
    with open(map_folder + "/report.pdf", 'rb') as f:
        mime = MIMEBase('report', 'pdf', filename='report.pdf')
        mime.add_header('Content-Disposition', 'attachment', filename='report.pdf')
        mime.add_header('Content-ID', '<0>')
        mime.add_header('X-Attachment-Id', '0')
        mime.set_payload(f.read())
        # 用Base64编码:
        encoders.encode_base64(mime)
        # 添加到MIMEMultipart:
        message.attach(mime)

    print ('sending email')
    print ('creating smtp object')
    try:
        smtpobj = smtplib.SMTP_SSL(host, port)
        print ('email success')
    except smtplib.SMTPException as e:
        print ('email failed', e)
    print ('login in')
    try:
        smtpobj.login(user, password)
    except smtplib.SMTPException as e:
        print ('email failed', e)
    print ('sending email')
    try:
        smtpobj.sendmail(sender, mapping_email_group, message.as_string())
        smtpobj.quit()
        print ('send email ok')
    except smtplib.SMTPException as e:
        print ('email failed', e)
