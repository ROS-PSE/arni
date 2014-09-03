from PySide.QtGui import QStandardItemModel, QStandardItem, QListView, QApplication, QSortFilterProxyModel
import sys

model = QStandardItemModel()


def is_clicked(index):
    print(model.itemFromIndex(index).text())

app = QApplication(sys.argv)
item = QStandardItem()
item.setText("Test")
model.appendRow(item)
view = QListView()
view.setMinimumSize(200, 200)

proxy_model = QSortFilterProxyModel()
proxy_model.setSourceModel(model)
view.setModel(proxy_model)
view.clicked.connect(is_clicked)
#this works fine
view.clicked.emit(model.indexFromItem(item))
# now click the Test entry manually, the program will crash

view.show()
app.exec_()