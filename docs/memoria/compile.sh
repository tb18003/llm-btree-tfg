xelatex main.tex 2>&1 >/dev/null
biber main 2>&1 >/dev/null
xelatex main.tex 2>&1 >/dev/null

echo "File 'main.pdf' created!"
echo "Cleaning..."

rm *.aux *.bbl *.bcf *.blg *.log *.out *.run.xml *.toc

echo "Finished!"