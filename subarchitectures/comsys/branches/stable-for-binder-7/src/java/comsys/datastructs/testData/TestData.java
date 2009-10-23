package comsys.datastructs.testData;

public class TestData {

	public TestData() {
		string = "";
	}
	
	public TestData(String string) {
		this.string = string;
	}
	
	String string ;
	
	public void setString(String string) {
		this.string = string;
	}
	

	
	public String getString() {
		return string;
	}
}
