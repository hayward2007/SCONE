OUTDIR := build
OUTPUT := output
EN_MAIN := root.tex
KO_MAIN := root_ko.tex
EN_BUILD := $(OUTDIR)/root.pdf
KO_BUILD := $(OUTDIR)/root_ko.pdf
EN_PDF := $(OUTPUT)/SCONE_ICRA_English.pdf
KO_PDF := $(OUTPUT)/SCONE_ICRA_Korean.pdf
FIGURES := figures/flat_metrics.pdf figures/stair_results.pdf figures/control_architecture.pdf

.PHONY: all figures check clean

all: $(EN_PDF) $(KO_PDF)

figures: $(FIGURES)

$(FIGURES): figures/generate_figures.py ../../benchmark/results/flat-nominal.jsonl ../../benchmark/results/stairs-nominal.jsonl
	python figures/generate_figures.py

$(EN_BUILD): $(EN_MAIN) build.sh references.bib $(wildcard sections/*.tex) ieeeconf.cls IEEEtran.bst $(FIGURES)
	@mkdir -p $(OUTDIR)
	./build.sh root $(OUTDIR)

$(KO_BUILD): $(KO_MAIN) build.sh references.bib $(wildcard sections_ko/*.tex) ieeeconf.cls IEEEtran.bst $(FIGURES)
	@mkdir -p $(OUTDIR)
	./build.sh root_ko $(OUTDIR)

$(EN_PDF): $(EN_BUILD)
	@mkdir -p $(OUTPUT)
	cp $(EN_BUILD) $(EN_PDF)

$(KO_PDF): $(KO_BUILD)
	@mkdir -p $(OUTPUT)
	cp $(KO_BUILD) $(KO_PDF)

check: all
	pdfinfo $(EN_PDF) | grep -E 'Pages|Page size|PDF version|Encrypted'
	pdfinfo $(KO_PDF) | grep -E 'Pages|Page size|PDF version|Encrypted'
	pdffonts $(EN_PDF)
	pdffonts $(KO_PDF)

clean:
	rm -f $(OUTDIR)/* $(OUTPUT)/* $(FIGURES)
