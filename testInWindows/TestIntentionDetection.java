public class TestIntentionDetection {
	static {		
		System.load("./IntentionDetection.dll");
	}
	
	private native long setCallback();
	private native boolean sendMessage(long addr, String data);
	public native String getCurrentSituation(long addr);
	
	public static void main(String[] args) {
		System.out.println("test intention detection library");
		
		TestIntentionDetection obj = new TestIntentionDetection();
		long addr = obj.setCallback();
		String data = "{\"messagetype\":1,\"objects\":[{\"id\":0,\"position\":[267.1981506347656,266.4982604980469],\"velocity\":[267.1981506347656,266.4982604980469]},{\"id\":0,\"position\":[394.8879089355469,380.6634826660156],\"velocity\":[394.8879089355469,380.6634826660156]},{\"id\":0,\"position\":[360.1019592285156,399.054443359375],\"velocity\":[360.1019592285156,399.054443359375]},{\"id\":0,\"position\":[388.0006408691406,436.0],\"velocity\":[388.0006408691406,436.0]},{\"id\":0,\"position\":[341.8468322753906,177.62918090820313],\"velocity\":[341.8468322753906,177.62918090820313]},{\"id\":0,\"position\":[357.6184997558594,297.4827880859375],\"velocity\":[357.6184997558594,297.4827880859375]},{\"id\":0,\"position\":[304.11572265625,331.4938049316406],\"velocity\":[304.11572265625,331.4938049316406]},{\"id\":0,\"position\":[298.0235290527344,141.53126525878906],\"velocity\":[298.0235290527344,141.53126525878906]},{\"id\":0,\"position\":[80.10202026367188,361.4310302734375],\"velocity\":[80.10202026367188,361.4310302734375]}],\"platforms\":[{\"id\":0,\"position\":[97.84223937988281,376.3471374511719],\"velocity\":[97.84223937988281,376.3471374511719]}],\"timestamp\":1703644906.3919785}";
		int n = 0;
		while (n < 10) {
			obj.sendMessage(addr, data);
			//System.out.println("send data " + data);
		
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		
			String res = obj.getCurrentSituation(addr);
			//System.out.println("recv data " + res);
			
			n++;
		}
		System.out.println("test intention detection library end");
	}
	
	private void callback(String data) {
		System.out.println("java callback " + data);
	}
}
