# Scrapy settings for scrapper project
#
# For simplicity, this file contains only the most important settings by
# default. All the other settings are documented here:
#
#     http://doc.scrapy.org/en/latest/topics/settings.html
#

import sys
sys.path.append('/home/julien/research_project/django_server')

import os
os.environ['DJANGO_SETTINGS_MODULE'] = 'django_server.settings'

BOT_NAME = 'scrapper'

SPIDER_MODULES = ['scrapper.spiders']
NEWSPIDER_MODULE = 'scrapper.spiders'

ITEM_PIPELINES = {
    'scrapper.pipelines.SketchupModelPipeline': 300,
}

# Crawl responsibly by identifying yourself (and your website) on the user-agent
#USER_AGENT = 'scrapper (+http://www.yourdomain.com)'
