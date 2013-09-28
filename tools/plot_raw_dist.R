args <- commandArgs(trailingOnly = TRUE)
dist <- read.table(args[1])[,1]
x <- seq(0,1,length=length(dist))
png(args[2])
plot(x,dist,type="l",xlab='Distance',ylab='Probability')
dev.off()
