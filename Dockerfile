FROM pymesh/pymesh

COPY parallelobox.py .

#ADD models/ models/

#RUN python pymesh_test.py
#RUN python boxslicer.py models/Cube10mm.stl
CMD ["python", "parallelobox.py", "/models/saturnv.stl"]

