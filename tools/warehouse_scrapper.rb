#!/usr/bin/env ruby

require 'nokogiri'	#to parse html
require 'open-uri'	
require 'optparse'	#to parse cmd line argument to implement
			#http://stackoverflow.com/questions/4244611/pass-variables-to-ruby-script-via-command-line

@main_url = "http://sketchup.google.com"
@names = []
@wait_max = 0 #random wait for robot detection

def download_from_url(url)
	puts "Accessing : " + url
	puts "---------------------------------------"
	page = Nokogiri::HTML open(url)
	page.css('div.searchresult').each do |div|
		name = div.attr 'id'
		link_skp = @main_url + div.css('a')[3].attr('href') if not div.css('a')[3].nil?
		link_img = @main_url + div.css('img').attr('src').value
		title_model = div.css('#bylinetitle a').attr('title').value
		tag_model = div.css('td > span.smaller').children
		tag_model = tag_model.map{|child| (child.text.include?("Download") ? "" : child.text)}.inject(&:+)
		tag_model.gsub!("\n", "")

		# puts "name : #{name}"
		# puts "link_skp : #{link_skp}"
		# puts "title_model : #{title_model}"
		# puts "tag_model : #{tag_model}"
		
		cmd = "wget -nc -q -O #{name}.skp '#{link_skp}';"
		cmd += "wget -nc -q -O #{name}.jpg '#{link_img}';"
		cmd += "echo \"title : #{title_model}\ntag : #{tag_model}\" > #{name}.txt;"
		
		puts "Get file #{name} -> " + system(cmd).to_s
		@names << name
		sleep @wait_max * rand
	end
end

#deprecated use the skp2tri instead
def convert_skps()
	outFile = File.new @download_folder + "files.txt", 'w'
	Dir[@download_folder + '*.skp'].each do |file|
		basename = File.basename(file, File.extname(file))
		
		outFile.puts basename + ".skp" if (not File.exist? @download_folder + basename + ".raw" and 
			`file -ib #{file}`.gsub(/\n/,"")=="application/octet-stream; charset=binary")
	end
	#launch remote script
	system @conv_to_tri
end

def research_keywords_url(keywords)
	@main_url + '/3dwarehouse/search?q=title%3A' + keywords + '+filetype%3Askp&styp=m&scoring=p&mfile=skp&btnG=Rechercher'
end

def research_similar_url(modeld_ref)
	@main_url + '/3dwarehouse/similar?mid=' + model_ref
end

keywords = ARGV.map{|p,n| p + "+"}.inject(&:+)
download_from_url research_keywords_url keywords[0,keywords.size-1]

