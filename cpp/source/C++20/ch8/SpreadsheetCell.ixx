module;

export module spreadsheet_cell;

import <string>;
import <string_view>;

export class SpreadsheetCell
{
public:
	SpreadsheetCell();									// 디폴트 생성자
	SpreadsheetCell(double initialValue);				// double 값을 명시적으로 정의
	SpreadsheetCell(std::string_view initialValue);		// 

	SpreadsheetCell(const SpreadsheetCell& src);		// 복제 생성자 

	void setValue(double value);
	double getValue() const;

	void setString(std::string_view value);
	std::string getString() const;

private:
	std::string doubleToString(double value) const;
	double stringToDouble(std::string_view value) const;

	double m_value{ 0 };
};
