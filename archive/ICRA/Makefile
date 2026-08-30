MAIN := root.tex
OUTDIR := build
PDF := $(OUTDIR)/root.pdf

.PHONY: all clean check

all: $(PDF)

$(PDF): $(MAIN) references.bib $(wildcard sections/*.tex) ieeeconf.cls IEEEtran.bst
	@mkdir -p $(OUTDIR)
	tectonic --keep-logs --keep-intermediates --outdir $(OUTDIR) $(MAIN)

check: $(PDF)
	pdfinfo $(PDF) | grep -E 'Pages|Page size|PDF version|Encrypted'
	pdffonts $(PDF)

clean:
	rm -f $(OUTDIR)/*
