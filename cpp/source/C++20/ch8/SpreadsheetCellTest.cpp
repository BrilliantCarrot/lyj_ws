import spreadsheet_cell;
import <iostream>;
import <memory>;

using namespace std;

int main()
{
	SpreadsheetCell myCell;
	myCell.setValue(6);
	cout << "cell 1: " << mycell.getValue() << endl;

	auto smartCellp { make_unique<SpreadsheetCell>() };
	//일반 포인터를 사용해도 되지만 권장하지 않는다.
	SpreadsheetCell* myCellp { new SpreadsheetCell {} };
	
	delete myCellp;
	myCellp = nullptr;
}
