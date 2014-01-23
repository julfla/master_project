from twisted.internet import reactor
from scrapy.crawler import Crawler
from scrapy.settings import Settings
from scrapy import log, signals
from scrapy.utils.project import get_project_settings


from scrapper import settings
from scrapper.spiders.model_spider import ModelSpider

spider = ModelSpider(domain="sketchup.google.com", 
	start_urls=["http://sketchup.google.com/3dwarehouse/search?q=mug&styp=m&scoring=t&btnG=Rechercher"])
settings = get_project_settings()
crawler = Crawler(settings)
crawler.signals.connect(reactor.stop, signal=signals.spider_closed)
crawler.configure()
crawler.crawl(spider)
crawler.start()
log.start()
reactor.run() # the script will block here
