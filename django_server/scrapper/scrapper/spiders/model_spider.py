# Connect to Django
# http://doc.scrapy.org/en/latest/topics/djangoitem.html
# http://stackoverflow.com/questions/19068308/access-django-models-with-scrapy-defining-path-to-django-project


from scrapy.spider import Spider
from scrapy.selector import Selector

from scrapper.items import ModelItem

class ModelSpider(Spider):
    name = "model"
    allowed_domains = ["http://sketchup.google.com"]
    start_urls = ["http://sketchup.google.com/3dwarehouse/search?q=bowl&styp=m&scoring=t&btnG=Rechercher"]

    # test url :
    # http://sketchup.google.com/3dwarehouse/details?mid=6a772d12b98ab61dc26651d9d35b77ca&prevstart=0
    def addItemToList(self, url):
    	self.start_urls.append(url)

    def parse(self, response):
        sel = Selector(response)
        models = []
        models_div = sel.xpath('//div[@class="searchresult"]')

        def extract_uft8(field):
            return field.extract()[0].encode('utf8')

        for model_div in models_div:
            model = ModelItem()
            site_basename = "http://sketchup.google.com"
            model['google_id'] = extract_uft8(model_div.xpath('@id'))
            model['link_image'] = site_basename + extract_uft8(model_div.
                xpath('table/tr/td/a/img/@src'))
            right_side_info = model_div.xpath('table/tr/td[@valign="top"]')
            model['title'] = extract_uft8(right_side_info.
                xpath('div/span[@id="bylinetitle"]/a/@title'))
            #model['link_full_description'] = right_side_info.
            #   xpath('div/span[@id="bylinetitle"]/a/@href').extract()
            model['link_skp'] = site_basename + extract_uft8(right_side_info.
                xpath('span/div/span/a[@class="dwnld"]/@href'))
            models.append(model)
        return models