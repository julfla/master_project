args <- commandArgs(trailingOnly = TRUE)

data <- read.csv(args[1], sep=" ")
library(ggplot2)
library(reshape)
library(scales)
data.m <- melt(data)
data.m <- ddply(data.m, .(variable), transform, rescale = rescale(value))
p <- ggplot(data.m, aes(variable, model)) + geom_tile(aes(fill = rescale), colour = "white") + scale_fill_gradient(low = "steelblue", high = "white")
base_size <- 9
p <- p + theme_grey(base_size = base_size) + labs(x = "", y = "") + scale_x_discrete(expand = c(0, 0)) + scale_y_discrete(expand = c(0, 0)) + theme(axis.ticks = element_blank(), axis.text.x = element_text(size = base_size * 0.8, angle = 270, hjust = 0, colour = "grey50"))

svg('rplot.svg')
plot(p)
dev.off()
